// sts_driver.cpp
#include "sts_driver.hpp"
#include <chrono>
#include <thread>
#include <stdexcept>
#include <iostream>
#include <cstring>  // For strerror

// Manual defines for ioctl flush (to avoid <termios.h>)
#ifndef TCFLSH
#define TCFLSH 0x540B  // Standard value from bits/termios.h
#endif
#ifndef TCIOFLUSH
#define TCIOFLUSH 2    // Flush both input and output
#endif
#ifndef BOTHER
#define BOTHER 0010000 // Custom baud flag from asm/termbits.h
#endif

ServoController::ServoController(const std::string& port, uint32_t baudrate, double timeout) {
    timeout_ms_ = static_cast<int>(timeout * 1000);

    // Open the serial port
    fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd_ < 0) {
        throw std::runtime_error("Failed to open serial port: " + port + " - " + std::string(strerror(errno)));
    }

    // Flush any existing data using ioctl
    ioctl(fd_, TCFLSH, TCIOFLUSH);

    // Stabilize USB (common for ttyUSB)
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Set custom baud and other params via termios2
    struct termios2 tio;
    if (ioctl(fd_, TCGETS2, &tio) < 0) {
        close();
        throw std::runtime_error("Failed to get termios2 settings: " + std::string(strerror(errno)) + " (errno=" + std::to_string(errno) + ")");
    }

    // Clear and set flags for 8N1, no flow control, custom baud
    tio.c_cflag &= ~(CBAUD | CSIZE | CSTOPB | PARENB | PARODD | CRTSCTS);
    tio.c_cflag |= BOTHER | CS8 | CLOCAL | CREAD;
    tio.c_iflag = IGNPAR;  // Ignore parity errors
    tio.c_oflag = 0;       // Raw output
    tio.c_lflag = 0;       // Raw input
    tio.c_ispeed = baudrate;
    tio.c_ospeed = baudrate;
    tio.c_cc[VMIN] = 1;    // Min chars to read
    tio.c_cc[VTIME] = 0;   // No inter-char timeout (we'll use select)

    if (ioctl(fd_, TCSETS2, &tio) < 0) {
        close();
        throw std::runtime_error("Failed to set custom baud rate (" + std::to_string(baudrate) + "): " + std::string(strerror(errno)) + " (errno=" + std::to_string(errno) + ")");
    }

    // Verify settings
    if (ioctl(fd_, TCGETS2, &tio) < 0 || tio.c_ispeed != baudrate || tio.c_ospeed != baudrate) {
        close();
        throw std::runtime_error("Baud rate verification failed: set to " + std::to_string(tio.c_ispeed) + "/" + std::to_string(tio.c_ospeed) + " instead of " + std::to_string(baudrate));
    }

    std::cout << "Serial port " << port << " opened at custom baud " << baudrate << " with POSIX termios." << std::endl;  // Debug
}

ServoController::~ServoController() {
    close();
}

void ServoController::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

uint8_t ServoController::calculate_checksum(const std::vector<unsigned char>& data) {
    uint16_t sum = 0;
    for (auto val : data) {
        sum += val;
    }
    return (~sum) & 0xFF;
}

void ServoController::send_packet(uint8_t servo_id, uint8_t command, const std::vector<unsigned char>& parameters) {
    std::vector<unsigned char> packet = {HEADER[0], HEADER[1]};
    packet.push_back(servo_id);
    packet.push_back(static_cast<uint8_t>(parameters.size() + 2));
    packet.push_back(command);
    packet.insert(packet.end(), parameters.begin(), parameters.end());
    
    std::vector<unsigned char> checksum_data(packet.begin() + 2, packet.end());
    packet.push_back(calculate_checksum(checksum_data));
    
    ioctl(fd_, TCFLSH, TCIOFLUSH);  // Flush IO buffers
    ssize_t written = write(fd_, packet.data(), packet.size());
    if (written != static_cast<ssize_t>(packet.size())) {
        throw std::runtime_error("Failed to write full packet: wrote " + std::to_string(written) + " bytes - " + std::string(strerror(errno)));
    }
}

std::optional<Response> ServoController::receive_packet() {
    unsigned char byte;
    // Search for the first header byte
    for (int i = 0; i < 50; ++i) {
        if (!timed_read(&byte, 1)) return std::nullopt;
        if (byte == 0xFF) {
            // Read the second header byte
            if (!timed_read(&byte, 1)) return std::nullopt;
            if (byte == 0xFF) goto header_found;
        }
    }
    return std::nullopt; // Header not found
header_found:;

    std::vector<unsigned char> header_and_len(2);
    if (!timed_read(header_and_len.data(), 2)) return std::nullopt;

    uint8_t servo_id = header_and_len[0];
    uint8_t length = header_and_len[1];

    if (length < 2) return std::nullopt;

    std::vector<unsigned char> remaining_data(length);
    if (!timed_read(remaining_data.data(), length)) return std::nullopt;

    uint8_t error = remaining_data[0];
    std::vector<unsigned char> parameters(remaining_data.begin() + 1, remaining_data.end() - 1);
    uint8_t received_checksum = remaining_data.back();

    std::vector<unsigned char> checksum_data = {servo_id, length, error};
    checksum_data.insert(checksum_data.end(), parameters.begin(), parameters.end());
    
    if (received_checksum != calculate_checksum(checksum_data)) return std::nullopt;

    return Response{servo_id, error, parameters};
}

bool ServoController::timed_read(unsigned char* buf, size_t len) {
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(fd_, &readfds);

    struct timeval tv;
    tv.tv_sec = timeout_ms_ / 1000;
    tv.tv_usec = (timeout_ms_ % 1000) * 1000;

    int ready = select(fd_ + 1, &readfds, nullptr, nullptr, &tv);
    if (ready < 0) {
        throw std::runtime_error("Select error: " + std::string(strerror(errno)));
    }
    if (ready == 0) return false;  // Timeout

    ssize_t bytes = read(fd_, buf, len);
    if (bytes < 0) {
        throw std::runtime_error("Read error: " + std::string(strerror(errno)));
    }
    if (bytes != static_cast<ssize_t>(len)) return false;  // Incomplete

    return true;
}

bool ServoController::ping(uint8_t servo_id, int retries) {
    std::lock_guard<std::mutex> lock(comm_lock_);
    for (int i = 0; i < retries; ++i) {
        send_packet(servo_id, CMD_PING);
        auto response = receive_packet();
        if (response && response->id == servo_id) {
            return true;
        }
    }
    return false;
}

// ... (The rest of the file remains unchanged from your previous version – all other methods like write_register, read_register, set_position, etc., are the same. No need to repeat them here as they don't contribute to the error.)

std::optional<Response> ServoController::write_register(uint8_t servo_id, uint8_t register_addr, uint16_t value, uint8_t num_bytes, bool expect_response) {
    std::vector<unsigned char> parameters = {register_addr};
    if (num_bytes == 1) {
        parameters.push_back(value & 0xFF);
    } else {
        parameters.push_back(value & 0xFF);
        parameters.push_back((value >> 8) & 0xFF);
    }
    std::lock_guard<std::mutex> lock(comm_lock_);
    uint8_t cmd = expect_response ? CMD_WRITE : CMD_REG_WRITE;
    send_packet(servo_id, cmd, parameters);
    if (expect_response) {
        return receive_packet();
    }
    return std::nullopt;
}

std::optional<int16_t> ServoController::read_register(uint8_t servo_id, uint8_t register_addr, uint8_t num_bytes) {
    std::vector<unsigned char> parameters = {register_addr, num_bytes};
    std::lock_guard<std::mutex> lock(comm_lock_);
    send_packet(servo_id, CMD_READ, parameters);
    auto response = receive_packet();
    if (!response || response->id != servo_id || response->error != 0 || response->parameters.size() != num_bytes) {
        return std::nullopt;
    }
    int16_t value = 0;
    if (num_bytes == 1) {
        value = response->parameters[0];
    } else {
        value = response->parameters[0] | (response->parameters[1] << 8);
    }
    return value;
}

bool ServoController::set_position(uint8_t servo_id, uint16_t position) {
    auto response = write_register(servo_id, REG_TARGET_POSITION, position, 2, true);
    return response.has_value() && response->error == 0;
}

std::optional<int16_t> ServoController::get_position(uint8_t servo_id) {
    return read_register(servo_id, REG_CURRENT_POSITION, 2);
}

bool ServoController::enable_torque(uint8_t servo_id, bool enable) {
    auto response = write_register(servo_id, REG_TORQUE_ENABLE, enable ? 1 : 0, 1, true);
    return response.has_value() && response->error == 0;
}

ServoStatus ServoController::get_servo_status(uint8_t servo_id) {
    ServoStatus status;
    status.load = read_register(servo_id, REG_CURRENT_LOAD, 2);
    status.current = read_register(servo_id, REG_CURRENT_CURRENT, 2);
    return status;
}

std::optional<bool> ServoController::is_moving(uint8_t servo_id) {
    auto moving_status = read_register(servo_id, REG_MOBILE_SIGN, 1);
    if (moving_status.has_value()) {
        return moving_status.value() == 1;
    }
    return std::nullopt;
}

bool ServoController::wait_for_stop(uint8_t servo_id, int timeout_seconds) {
    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(timeout_seconds)) {
        auto moving = is_moving(servo_id);
        if (moving.has_value() && !moving.value()) {
            return true; // Not moving
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return false; // Timed out
}

void ServoController::sync_write_positions(const std::map<uint8_t, uint16_t>& servo_positions) {
    if (servo_positions.empty()) return;
    std::lock_guard<std::mutex> lock(comm_lock_);
    
    std::vector<unsigned char> parameters = {REG_TARGET_POSITION, 2}; // Start Addr, Bytes per servo
    for (const auto& [id, position] : servo_positions) {
        parameters.push_back(id);
        parameters.push_back(position & 0xFF);
        parameters.push_back((position >> 8) & 0xFF);
    }
    send_packet(BROADCAST_ID, CMD_SYNC_WRITE, parameters);
}

void ServoController::sync_write_register_values(uint8_t register_addr, uint8_t num_bytes, const std::map<uint8_t, uint16_t>& servo_values) {
    if (servo_values.empty()) return;
    std::lock_guard<std::mutex> lock(comm_lock_);

    std::vector<unsigned char> parameters = {register_addr, num_bytes};
    for (const auto& [id, value] : servo_values) {
        parameters.push_back(id);
        if (num_bytes == 1) {
            parameters.push_back(value & 0xFF);
        } else {
            parameters.push_back(value & 0xFF);
            parameters.push_back((value >> 8) & 0xFF);
        }
    }
    send_packet(BROADCAST_ID, CMD_SYNC_WRITE, parameters);
}

ServoController::ChangeIdResult ServoController::change_servo_id_safe(uint8_t current_id, uint8_t new_id) {
    if (!ping(current_id)) {
        return {false, "", {"Servo " + std::to_string(current_id) + " not responding."}};
    }
    if (ping(new_id)) {
        return {false, "", {"ID " + std::to_string(new_id) + " is already in use."}};
    }

    auto unlock_response = write_register(current_id, REG_EEPROM_LOCK, 0, 1, true);
    if (!unlock_response || unlock_response->error != 0) {
        return {false, "", {"Failed to unlock EEPROM on servo " + std::to_string(current_id) + "."}};
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    write_register(current_id, REG_ID, new_id, 1, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    auto lock_response = write_register(new_id, REG_EEPROM_LOCK, 1, 1, true);
    if (!lock_response || lock_response->error != 0) {
        return {false, "", {"Failed to lock EEPROM on new ID " + std::to_string(new_id) + ". Change may not be permanent."}};
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    if (ping(new_id) && !ping(current_id)) {
        return {true, "Successfully changed ID from " + std::to_string(current_id) + " to " + std::to_string(new_id) + ".", {}};
    }
    return {false, "", {"Final verification failed. The ID may be in an inconsistent state."}};
}

std::vector<uint8_t> ServoController::fast_scan_servos(uint8_t start_id, uint8_t end_id) {
    int original_timeout = timeout_ms_;
    std::vector<uint8_t> found_servos;
    timeout_ms_ = 10; // 10ms for fast scan
    for (uint8_t id = start_id; id <= end_id; ++id) {
        if (ping(id, 1)) { // Only 1 retry for speed
            found_servos.push_back(id);
        }
    }
    timeout_ms_ = original_timeout;
    return found_servos;
}