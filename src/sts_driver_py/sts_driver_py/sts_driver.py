import serial
import time
import struct
import threading

class ServoController:
    # Protocol constants
    HEADER = [0xFF, 0xFF]
    BROADCAST_ID = 0xFE
    
    # Command codes
    CMD_PING = 0x01
    CMD_READ = 0x02
    CMD_WRITE = 0x03
    CMD_REG_WRITE = 0x04
    CMD_ACTION = 0x05
    CMD_RESET = 0x06
    CMD_SYNC_WRITE = 0x83
    
    # Register addresses
    REG_ID = 0x05
    REG_BAUD_RATE = 0x06
    REG_TORQUE_ENABLE = 0x28
    REG_TARGET_POSITION = 0x2A
    REG_RUNNING_SPEED = 0x2E
    REG_CURRENT_POSITION = 0x38
    REG_CURRENT_LOAD = 0x3C
    REG_MOBILE_SIGN = 0x42
    REG_CURRENT_CURRENT = 0x45
    REG_EEPROM_LOCK = 55
    
    def __init__(self, port, baudrate=1000000, timeout=0.1):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        self.ser.flushInput()
        self.ser.flushOutput()
        self._comm_lock = threading.Lock()

    def _calculate_checksum(self, data):
        return (~sum(data)) & 0xFF

    def _send_packet(self, servo_id, command, parameters=None):
        if parameters is None:
            parameters = []
        packet = self.HEADER.copy()
        packet.append(servo_id)
        length = len(parameters) + 2
        packet.append(length)
        packet.append(command)
        packet.extend(parameters)
        checksum_data = packet[2:]
        checksum = self._calculate_checksum(checksum_data)
        packet.append(checksum)
        self.ser.flushInput()
        self.ser.write(bytes(packet))

    def _receive_packet(self):
        byte1 = self.ser.read(1)
        if not byte1: return None 

        attempts = 0
        while byte1[0] != 0xFF and attempts < 50:
            byte1 = self.ser.read(1)
            if not byte1: return None
            attempts += 1
        
        if byte1[0] != 0xFF: return None

        byte2 = self.ser.read(1)
        if not byte2 or byte2[0] != 0xFF: return None

        id_byte = self.ser.read(1)
        length_byte = self.ser.read(1)
        if not id_byte or not length_byte: return None
        
        servo_id, length = id_byte[0], length_byte[0]
        
        remaining_data = self.ser.read(length)
        if len(remaining_data) != length: return None
            
        error = remaining_data[0]
        parameters = list(remaining_data[1:-1]) if length > 2 else []
        received_checksum = remaining_data[-1]
        
        checksum_data = [servo_id, length, error] + parameters
        calculated_checksum = self._calculate_checksum(checksum_data)
        
        if received_checksum != calculated_checksum: return None
        
        return {'id': servo_id, 'error': error, 'parameters': parameters}

    def ping(self, servo_id, retries=1):
        with self._comm_lock:
            for _ in range(retries):
                self._send_packet(servo_id, self.CMD_PING)
                response = self._receive_packet()
                if response is not None and response['id'] == servo_id:
                    return True
            return False

    def write_register(self, servo_id, register_addr, value, num_bytes=1, expect_response=True):
        """
        Writes a value to a register. Includes a flag to control whether to wait for a response.
        This is critical for commands like changing an ID, where the servo won't respond on its old ID.
        """
        with self._comm_lock:
            if num_bytes == 1:
                parameters = [register_addr, value & 0xFF]
            else:
                parameters = [register_addr, value & 0xFF, (value >> 8) & 0xFF]
            
            self._send_packet(servo_id, self.CMD_WRITE, parameters)
            
            if expect_response and servo_id != self.BROADCAST_ID:
                return self._receive_packet()
            return None

    def read_register(self, servo_id, register_addr, num_bytes=1):
        with self._comm_lock:
            self._send_packet(servo_id, self.CMD_READ, [register_addr, num_bytes])
            response = self._receive_packet()
            if response is None or response['error'] != 0: return None
            params = response['parameters']
            if num_bytes == 1:
                return params[0] if params else None
            return params[0] + (params[1] << 8) if len(params) >= 2 else None

    def set_position(self, servo_id, position, speed=None, acceleration=None):
        if speed is not None:
            self.write_register(servo_id, self.REG_RUNNING_SPEED, speed, 2)
        return self.write_register(servo_id, self.REG_TARGET_POSITION, position, 2)

    def set_torque_enable(self, servo_id, enable):
        return self.write_register(servo_id, self.REG_TORQUE_ENABLE, 1 if enable else 0, 1)

    def get_position(self, servo_id):
        return self.read_register(servo_id, self.REG_CURRENT_POSITION, 2)

    def get_servo_status(self, servo_id):
        status = {}
        status['load'] = self.read_register(servo_id, self.REG_CURRENT_LOAD, 2)
        status['current'] = self.read_register(servo_id, self.REG_CURRENT_CURRENT, 2)
        return status
        
    def is_moving(self, servo_id):
        moving_status = self.read_register(servo_id, self.REG_MOBILE_SIGN, 1)
        return moving_status == 1 if moving_status is not None else None

    def wait_for_stop(self, servo_id, timeout_seconds=5):
        start_time = time.time()
        while time.time() - start_time < timeout_seconds:
            if not self.is_moving(servo_id): return True
            time.sleep(0.05)
        return False

    def sync_write_positions(self, servo_positions):
        """
        Correctly sends a SYNC_WRITE command to move multiple servos to target positions.
        """
        if not servo_positions: return
        with self._comm_lock:
            # Parameters for SYNC_WRITE: Start Address (position), Bytes per servo (2)
            parameters = [self.REG_TARGET_POSITION, 2] 
            for servo_id, position in servo_positions.items():
                # Append [ID, Pos_Low, Pos_High] for each servo
                parameters.extend([servo_id, position & 0xFF, (position >> 8) & 0xFF])
            self._send_packet(self.BROADCAST_ID, self.CMD_SYNC_WRITE, parameters)

    def sync_write_register_values(self, register_addr, num_bytes, servo_values):
        if not servo_values: return
        with self._comm_lock:
            parameters = [register_addr, num_bytes]
            for servo_id, value in servo_values.items():
                parameters.append(servo_id)
                if num_bytes == 1:
                    parameters.append(value & 0xFF)
                else:
                    parameters.extend([value & 0xFF, (value >> 8) & 0xFF])
            self._send_packet(self.BROADCAST_ID, self.CMD_SYNC_WRITE, parameters)

    def change_servo_id_safe(self, current_id, new_id):
        """
        Permanently changes a servo's ID with step-by-step verification and deadlock prevention.
        """
        if not self.ping(current_id):
            return {'success': False, 'errors': [f"Servo {current_id} not responding."]}
        if self.ping(new_id):
            return {'success': False, 'errors': [f"ID {new_id} is already in use."]}

        unlock_response = self.write_register(current_id, self.REG_EEPROM_LOCK, 0, 1, expect_response=True)
        if not unlock_response or unlock_response.get('error', -1) != 0:
            return {'success': False, 'errors': [f"Failed to unlock EEPROM on servo {current_id}."]}
        time.sleep(0.05)

        self.write_register(current_id, self.REG_ID, new_id, 1, expect_response=False)
        time.sleep(0.1)

        lock_response = self.write_register(new_id, self.REG_EEPROM_LOCK, 1, 1, expect_response=True)
        if not lock_response or lock_response.get('error', -1) != 0:
            return {'success': False, 'errors': [f"Failed to lock EEPROM on new ID {new_id}. Change may not be permanent."]}
        time.sleep(0.1)

        if self.ping(new_id) and not self.ping(current_id):
            return {'success': True, 'message': f"Successfully changed ID from {current_id} to {new_id}."}
        else:
            return {'success': False, 'errors': ["Final verification failed. The ID may be in an inconsistent state."]}
    def enable_torque(self, servo_id, enable):
        """Enable or disable torque for a servo."""
        value = 1 if enable else 0
        return self.write_register(servo_id, self.REG_TORQUE_ENABLE, value, 1, expect_response=True)
    
    def fast_scan_servos(self, start_id=1, end_id=50):
        original_timeout = self.ser.timeout
        found_servos = []
        try:
            self.ser.timeout = 0.01 
            for servo_id in range(start_id, end_id + 1):
                if self.ping(servo_id):
                    found_servos.append(servo_id)
        finally:
            self.ser.timeout = original_timeout
        
        return found_servos

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
