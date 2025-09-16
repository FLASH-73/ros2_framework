// sts_driver.hpp
#ifndef STS_DRIVER_HPP_
#define STS_DRIVER_HPP_

#include <string>
#include <vector>
#include <mutex>
#include <optional>
#include <map>
#include <cstdint>
#include <fcntl.h>      // For open flags
#include <unistd.h>     // For close, read, write
#include <sys/ioctl.h>  // For ioctl
#include "asm_termios_proxy.h"  // Proxy for custom baud (replaces <asm/termios.h>)
#include <sys/select.h> // For timeout handling
#include <errno.h>      // For errno

// Struct for the response from a servo after a command
struct Response {
    uint8_t id;
    uint8_t error;
    std::vector<unsigned char> parameters;
};

// Struct to hold status information from a servo
struct ServoStatus {
    std::optional<int16_t> load;
    std::optional<int16_t> current;
};

class ServoController {
public:
    ServoController(const std::string& port, uint32_t baudrate = 1000000, double timeout = 0.1);
    ~ServoController();

    // Core Communication
    bool ping(uint8_t servo_id, int retries = 1);
    std::optional<Response> write_register(uint8_t servo_id, uint8_t register_addr, uint16_t value, uint8_t num_bytes = 1, bool expect_response = true);
    std::optional<int16_t> read_register(uint8_t servo_id, uint8_t register_addr, uint8_t num_bytes = 1);

    // High-Level Control
    bool set_position(uint8_t servo_id, uint16_t position);
    std::optional<int16_t> get_position(uint8_t servo_id);
    bool enable_torque(uint8_t servo_id, bool enable);
    void sync_write_positions(const std::map<uint8_t, uint16_t>& servo_positions);
    void sync_write_register_values(uint8_t register_addr, uint8_t num_bytes, const std::map<uint8_t, uint16_t>& servo_values);

    // Status and Utility Functions
    ServoStatus get_servo_status(uint8_t servo_id);
    std::optional<bool> is_moving(uint8_t servo_id);
    bool wait_for_stop(uint8_t servo_id, int timeout_seconds = 5);
    std::vector<uint8_t> fast_scan_servos(uint8_t start_id = 1, uint8_t end_id = 50);
    
    // Configuration
    struct ChangeIdResult {
        bool success;
        std::string message;
        std::vector<std::string> errors;
    };
    ChangeIdResult change_servo_id_safe(uint8_t current_id, uint8_t new_id);

    // Resource Management
    void close();

private:
    int fd_;  // Serial file descriptor
    std::mutex comm_lock_;
    int timeout_ms_;

    // Packet Handling
    uint8_t calculate_checksum(const std::vector<unsigned char>& data);
    void send_packet(uint8_t servo_id, uint8_t command, const std::vector<unsigned char>& parameters = {});
    std::optional<Response> receive_packet();

    // Helper for timed read
    bool timed_read(unsigned char* buf, size_t len);

    // Protocol Constants
    static constexpr uint8_t HEADER[2] = {0xFF, 0xFF};
    static constexpr uint8_t BROADCAST_ID = 0xFE;

    // Command codes
    static constexpr uint8_t CMD_PING = 0x01;
    static constexpr uint8_t CMD_READ = 0x02;
    static constexpr uint8_t CMD_WRITE = 0x03;
    static constexpr uint8_t CMD_REG_WRITE = 0x04;
    static constexpr uint8_t CMD_ACTION = 0x05;
    static constexpr uint8_t CMD_RESET = 0x06;
    static constexpr uint8_t CMD_SYNC_WRITE = 0x83;

    // Register addresses
    static constexpr uint8_t REG_ID = 0x05;
    static constexpr uint8_t REG_BAUD_RATE = 0x06;
    static constexpr uint8_t REG_TORQUE_ENABLE = 0x28;
    static constexpr uint8_t REG_TARGET_POSITION = 0x2A;
    static constexpr uint8_t REG_RUNNING_SPEED = 0x2E;
    static constexpr uint8_t REG_CURRENT_POSITION = 0x38;
    static constexpr uint8_t REG_CURRENT_LOAD = 0x3C;
    static constexpr uint8_t REG_MOBILE_SIGN = 0x42;
    static constexpr uint8_t REG_CURRENT_CURRENT = 0x45;
    static constexpr uint8_t REG_EEPROM_LOCK = 55;
};

#endif  // STS_DRIVER_HPP_