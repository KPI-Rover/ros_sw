import socketserver
import struct
import logging

logging.basicConfig(level=logging.INFO)

# Map command IDs to expected additional bytes: {cmd: additional_bytes}
# 0x01: get_api_version => 1 more byte (driver version)
# 0x02: set_motor_speed   => 1 (motor_id) + 4 (speed) = 5 bytes
# 0x03: set_all_motors_speed => 16 bytes (4 motors * 4 bytes each)
# 0x04: get_encoder       => 1 byte (motor_id)
# 0x05: get_all_encoders  => 0 extra bytes
CMD_LENGTHS = {
    0x01: 1,
    0x02: 5,
    0x03: 16,
    0x04: 1,
    0x05: 0,
}

class ECUEmulatorHandler(socketserver.BaseRequestHandler):
    def handle(self):
        while True:
            # Read command id (1 byte)
            header = self.request.recv(1)
            if not header:
                break
            cmd_id = header[0]
            expected = CMD_LENGTHS.get(cmd_id)
            if expected is None:
                logging.warning(f"Unknown command: {cmd_id:#04x}")
                continue
            data = b''
            while len(data) < expected:
                chunk = self.request.recv(expected - len(data))
                if not chunk:
                    break
                data += chunk

            logging.info(f"Received command {cmd_id:#04x} with data: {data.hex()}")

            # Process commands
            if cmd_id == 0x01:
                # get_api_version
                # Request: [1 byte driver version]
                driver_version = data[0]
                logging.info(f"Driver version: {driver_version}")
                # Response: [cmd_id, API_version]
                response = struct.pack("BB", 0x01, 1)  # API version hard-coded as 1
            elif cmd_id == 0x02:
                # set_motor_speed
                # Request: [motor_id (1 byte), speed (4 byte signed int)]
                motor_id, speed = struct.unpack(">Bi", data)
                logging.info(f"Set motor {motor_id} speed to {speed}")
                # Response: [cmd_id]
                response = struct.pack("B", 0x02)
            elif cmd_id == 0x03:
                # set_all_motors_speed
                # Request: 4 speeds (each 4 byte signed int)
                speeds = struct.unpack(">4i", data)
                logging.info(f"Set all motors speed to {speeds}")
                # Response: [cmd_id]
                response = struct.pack("B", 0x03)
            elif cmd_id == 0x04:
                # get_encoder
                # Request: [motor_id (1 byte)]
                motor_id = data[0]
                # Emulate encoder value (e.g., fixed value or computed)
                encoder_value = 1234
                logging.info(f"Get encoder for motor {motor_id}: {encoder_value}")
                # Response: [cmd_id, encoder_value (4 byte signed int)]
                response = struct.pack(">Bi", 0x04, encoder_value)
            elif cmd_id == 0x05:
                # get_all_encoders
                # Emulate encoder values for all four motors
                encoder_values = (1234, 2345, 3456, 4567)
                logging.info(f"Get all encoders: {encoder_values}")
                # Response: [cmd_id, 4 x 4 byte signed ints]
                response = struct.pack(">B4i", 0x05, *encoder_values)
            else:
                # Should not happen
                continue

            self.request.sendall(response)
            logging.info(f"Sent response: {response.hex()}")

if __name__ == "__main__":
    HOST, PORT = "localhost", 6000
    with socketserver.ThreadingTCPServer((HOST, PORT), ECUEmulatorHandler) as server:
        logging.info(f"ECU Emulator server running on {HOST}:{PORT}")
        server.serve_forever()
