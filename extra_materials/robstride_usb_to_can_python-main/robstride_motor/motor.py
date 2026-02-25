from .types import (Type0Message, Type18Message, Type3Message, TypeMessage,
                    Type17Message, Type24Message, Type1Message, Type4Message)
from .protocol import RobstrideProtocolWrapper
from serial import Serial
import time
import numpy as np
import struct


class RobstrideMotor:
    def __init__(self, port: str, baudrate: int, host_id: int = 253):
        self.port = port
        self.baudrate = baudrate
        self.protocol = RobstrideProtocolWrapper()
        self.motor_can_id = None
        self.host_id = host_id
        self.init_serial_connection()
        self.pos = 0.0
        self.vel = 0.0
        self.last_message_print_time = 0

    def init_serial_connection(self):
        self.serial = Serial(self.port, self.baudrate, timeout=0.1)
        print(f"Connected to {self.port} at {self.baudrate} baud.")

    def set_motor_id(self, motor_id: int):
        self.motor_can_id = motor_id

    def search_can_id(self):
        for n in range(255):
            print(f"Probing motor CAN ID: {n}", end='\r')
            message = Type0Message(
                host_id=self.host_id,
                motor_id=n
            )
            encoded = message.encode_message()
            self.send_message(encoded)
            time.sleep(0.01)
            response = self.read_response()
            if response is not None:
                print(response)
                print(f"Found motor CAN ID: {n}, saving...")
                #response comes from motor with its host_id
                self.set_motor_id(response.host_id)
                return

        print("No motor responded to CAN ID probe.")
        raise RuntimeError("Motor CAN ID not found.")

    def set_csp_mode(self):
        if self.motor_can_id is None:
            raise RuntimeError("Motor CAN ID not set.")
        print("Setting CSP mode...")
        message = Type18Message(
            host_id=self.host_id,
            motor_id=self.motor_can_id,
            index=b'\x05\x70',
            parameter_data=b'\x05\x00\x00\x00'
        )

        encoded = message.encode_message()
        self.send_message(encoded)
        time.sleep(0.01)
        response = self.read_response()
        if response is not None:
            if response.type_id == 2:
                print("CSP mode set.")
            else:
                raise RuntimeError("Unexpected response type "
                                   "when setting CSP mode.")

    def enable_movement(self):
        if self.motor_can_id is None:
            raise RuntimeError("Motor CAN ID not set.")
        print("Enabling motor movement...")
        message = Type3Message(
            host_id=self.host_id,
            motor_id=self.motor_can_id
        )
        encoded = message.encode_message()
        print(f"Encoded enable movement message: {encoded.hex()}")
        self.send_message(encoded)
        time.sleep(0.01)
        response = self.read_response()
        if response is not None:
            if response.type_id == 2:
                print("Motor movement enabled.")
            else:
                raise RuntimeError("Unexpected response type "
                                   "when enabling movement.")
            
    def disable_movement(self):
        print("Disabling motor movement...")
        message = Type4Message(
            host_id=self.host_id,
            motor_id=self.motor_can_id
        )
        encoded = message.encode_message()
        print(f"Encoded disable movement message: {encoded.hex()}")
        self.send_message(encoded)

    def set_max_ang_vel(self, ang_vel: float):
        if self.motor_can_id is None:
            raise RuntimeError("Motor CAN ID not set.")
        if ang_vel < 0.0 or ang_vel > np.pi:
            # print warning and clamp
            print("Warning: Angular velocity out of range (0 to 3.14 rad/s).")
            ang_vel = max(0.0, min(np.pi, ang_vel))
        print(f"Setting max angular velocity to {ang_vel} rad/s...")
        message = Type18Message(
            host_id=253,
            motor_id=self.motor_can_id,
            index=b'\x17\x70',
            parameter_data=struct.pack('<f', ang_vel)
        )

        encoded = message.encode_message()
        self.send_message(encoded)
        time.sleep(0.01)
        response = self.read_response()
        if response is not None:
            if response.type_id == 2:
                print("Max angular velocity set.")
            else:
                raise RuntimeError("Unexpected response type when"
                                   " setting max angular velocity.")

    def read_max_ang_vel(self) -> float:
        if self.motor_can_id is None:
            raise RuntimeError("Motor CAN ID not set.")
        print("Reading max angular velocity...")
        message = Type17Message(
            host_id=self.host_id,
            motor_id=self.motor_can_id,
            index=b'\x17\x70',
            parameter_data=b'\x00\x00\x00\x00'
        )

        encoded = message.encode_message()
        self.send_message(encoded)
        time.sleep(0.01)
        response = self.read_response()
        if response is None:
            raise RuntimeError("No response received when reading"
                               " max angular velocity.")

        if response.type_id != 17:
            raise RuntimeError("Unexpected response type when"
                               " reading max angular velocity.")

        return struct.unpack('<f', response.parameter_data)[0]
                
    def operation_control(self, feed_forward, pos, vel, kp, kd):
        if self.motor_can_id is None:
            raise RuntimeError("Motor CAN ID not set.")
        # print(f"Sending operation control command...")
        message = Type1Message(
            host_id=self.host_id,
            motor_id=self.motor_can_id,
            feed_forward=feed_forward,
            pos=pos,
            vel=vel,
            kp=kp,
            kd=kd
        )

        encoded = message.encode_message()
        self.send_message(encoded)
        time.sleep(0.001)
        response = self.read_response()
        if response is not None:
            self.pos = response.current_angle
            self.vel = response.current_angular_velocity
        # if response is not None:
        #     if response.type_id == 2:
        #         print("Operation control command sent.")
        #     else:
        #         raise RuntimeError("Unexpected response type when"
        #                            " sending operation control command.")

    def start_csp_mode(self):
        if self.motor_can_id is None:
            raise RuntimeError("Motor CAN ID not set.")
        print("Start CSP mode...")
        message = Type18Message(
            host_id=self.host_id,
            motor_id=self.motor_can_id,
            index=b'\x05\x70',
            parameter_data=b'\x05\x01\x00\x00'
        )

        encoded = message.encode_message()
        self.send_message(encoded)
        time.sleep(0.01)
        response = self.read_response()
        if response is not None:
            if response.type_id == 2:
                print("CSP mode started.")
            else:
                raise RuntimeError("Unexpected response type "
                                   "when enabling CSP mode.")

    def read_response(self) -> TypeMessage | None:
        if self.serial.in_waiting > 0:
            raw_data = self.serial.read(self.serial.in_waiting)
            # print(f"Raw data received: {raw_data.hex()}")
            messages = self.protocol.decode_all_messages(raw_data)
            
            if not messages:
                return None
            
            # Print messages once per second
            current_time = time.time()
            if current_time - self.last_message_print_time >= 1.0:
                if len(messages) > 1:
                    print(f"Warning: Received {len(messages)} messages at once!")
                for i, msg in enumerate(messages):
                    print(f"  Message {i+1}: {msg}")
                self.last_message_print_time = current_time
            
            # Return the first message (main response)
            return messages[0]

    def send_message(self, encoded_message: bytes):
        self.serial.write(encoded_message)

    def read_ang_pos(self) -> float:
        if self.motor_can_id is None:
            raise RuntimeError("Motor CAN ID not set.")
        message = Type17Message(
            host_id=self.host_id,
            motor_id=self.motor_can_id,
            index=b'\x16\x70',
            parameter_data=b'\x00\x00\x00\x00'
        )

        encoded = message.encode_message()
        self.send_message(encoded)
        time.sleep(0.003)
        response = self.read_response()
        if response is None:
            raise RuntimeError("No response received when reading"
                               " angular position.")
        if response.type_id != 17:
            raise RuntimeError("Unexpected response type when"
                               " reading angular position.")

        return struct.unpack('<f', response.parameter_data)[0]

    def read_mech_vel(self) -> float:
        if self.motor_can_id is None:
            raise RuntimeError("Motor CAN ID not set.")
        message = Type17Message(
            host_id=self.host_id,
            motor_id=self.motor_can_id,
            index=b'\x1B\x70',
            parameter_data=b'\x00\x00\x00\x00'
        )

        encoded = message.encode_message()
        self.send_message(encoded)
        time.sleep(0.003)
        response = self.read_response()
        if response is None:
            raise RuntimeError("No response received when reading"
                               " angular position.")
        if response.type_id != 17:
            raise RuntimeError("Unexpected response type when"
                               " reading angular position.")

        return struct.unpack('<f', response.parameter_data)[0]


    def set_stream_frequency(self, hz: int):
        if self.motor_can_id is None:
            raise RuntimeError("Motor CAN ID not set.")
        print(f"Starting frame stream at {hz} Hz...")
        interval_ms = int(1000 / hz)

        if interval_ms < 10 or interval_ms > 100:
            raise ValueError("Frame stream interval must be between"
                             " 10 ms and 100 ms.")

        if (interval_ms - 10) % 5 != 0:
            raise ValueError("Frame stream interval must be in"
                             " increments of 5 ms starting from 10 ms.")

        # 1 indicates 10 ms. Plus 1 increments by 5 ms
        unit = 1 + (interval_ms - 10) // 5

        message = Type18Message(
            host_id=self.host_id,
            motor_id=self.motor_can_id,
            index=b'\x26\x70',
            parameter_data=unit.to_bytes(4, 'big')
        )

        encoded = message.encode_message()
        self.send_message(encoded)
        time.sleep(0.01)
        response = self.read_response()
        if response is not None:
            if response.type_id == 2:
                print("Stream frequency set.")
            else:
                raise RuntimeError("Unexpected response type when"
                                   " setting stream frequency.")

    def start_frame_stream(self):
        if self.motor_can_id is None:
            raise RuntimeError("Motor CAN ID not set.")
        print("Starting frame stream at default rate...")
        message = Type24Message(
            host_id=self.host_id,
            motor_id=self.motor_can_id
        )
        encoded = message.encode_message()
        print(f"Encoded frame stream start message: {encoded.hex()}")
        self.send_message(encoded)
        time.sleep(0.01)
        response = self.read_response()
        if response is not None:
            print(response)
