from dataclasses import dataclass
from .mapping import (map_u16_to_angle,
                      map_u16_to_angular_vel,
                      map_u16_to_torque,
                      map_u16_to_temp,
                      map_torque_to_u16,
                      map_angle_to_u16,
                      map_angular_vel_to_u16,
                      map_Kp_to_u16,
                      map_Kd_to_u16)
from abc import ABC


@dataclass
class TypeMessage(ABC):
    type_id: int
    host_id: int
    motor_id: int
    extra_info: int = 0

    HEADER = b"AT"
    FOOTER = b"\x0d\x0a"
    LSB_BITS = 0b100

    def build_header(self) -> bytes:
        # 4 bytes sin empaquetar
        raw_header = bytes([
            self.type_id & 0xFF,
            self.extra_info & 0xFF,
            self.host_id & 0xFF,
            self.motor_id & 0xFF,
        ])

        # a entero
        header_val = int.from_bytes(raw_header, "big")

        # <<3 y agregar bits LSB obligatorios
        header_shifted = ((header_val << 3) | self.LSB_BITS) & 0xFFFFFFFF

        # volver a bytes
        return header_shifted.to_bytes(4, "big")


@dataclass
class Type18Message(TypeMessage):
    index: bytes = b""
    parameter_data: bytes = b""

    def __init__(
            self, motor_id: int, host_id: int, index: bytes,
            parameter_data: bytes):
        super().__init__(type_id=0x12, host_id=host_id, motor_id=motor_id)
        self.index = index
        self.parameter_data = parameter_data

    def encode_message(self) -> bytes:
        header = self.build_header()
        body = b'\x08' + self.index + b'\x00\x00' + self.parameter_data
        return self.HEADER + header + body + self.FOOTER


@dataclass
class Type2Message(TypeMessage):
    uncalibrated: int = 0
    gridlock: int = 0
    magnetic_coding_fault: int = 0
    overtemperature: int = 0
    overcurrent: int = 0
    undervoltage: int = 0
    mode_status: int = 0
    current_angle: float = 0.0
    current_angular_velocity: float = 0.0
    current_torque: float = 0.0
    current_temperature: float = 0.0

    def __init__(self,
                 host_id: int,
                 motor_id: int,
                 extra_info: int,
                 uncalibrated: int,
                 gridlock: int,
                 magnetic_coding_fault: int,
                 overtemperature: int,
                 overcurrent: int,
                 undervoltage: int,
                 mode_status: int,
                 current_angle: float,
                 current_angular_velocity: float,
                 current_torque: float,
                 current_temperature: float):

        super().__init__(
            type_id=0x02,
            host_id=host_id,
            motor_id=motor_id,
            extra_info=extra_info
        )

        self.uncalibrated = uncalibrated
        self.gridlock = gridlock
        self.magnetic_coding_fault = magnetic_coding_fault
        self.overtemperature = overtemperature
        self.overcurrent = overcurrent
        self.undervoltage = undervoltage
        self.mode_status = mode_status

        self.current_angle = current_angle
        self.current_angular_velocity = current_angular_velocity
        self.current_torque = current_torque
        self.current_temperature = current_temperature

    @classmethod
    def from_decoded_fields(
            cls, header: bytes, payload: bytes
    ) -> "Type2Message":
        """
        Produce un Type2Message desde header (4 bytes ya des-shifteados)
        y payload (8 bytes).
        """

        extra_info = header[1]
        host_id = header[2]
        motor_id = header[3]

        uncalibrated = (extra_info >> 7) & 0x01
        gridlock = (extra_info >> 6) & 0x01
        magnetic_coding_fault = (extra_info >> 5) & 0x01
        overtemperature = (extra_info >> 4) & 0x01
        overcurrent = (extra_info >> 3) & 0x01
        undervoltage = (extra_info >> 2) & 0x01
        mode_status = extra_info & 0x03

        current_ang_raw = int.from_bytes(payload[0:2], 'big')
        current_ang_vel_raw = int.from_bytes(payload[2:4], 'big')
        current_torque_raw = int.from_bytes(payload[4:6], 'big')
        current_temp_raw = int.from_bytes(payload[6:8], 'big')

        current_angle = map_u16_to_angle(current_ang_raw)
        current_angular_velocity = map_u16_to_angular_vel(current_ang_vel_raw)
        current_torque = map_u16_to_torque(current_torque_raw)
        current_temperature = map_u16_to_temp(current_temp_raw)

        return cls(
            host_id=host_id,
            motor_id=motor_id,
            extra_info=extra_info,
            uncalibrated=uncalibrated,
            gridlock=gridlock,
            magnetic_coding_fault=magnetic_coding_fault,
            overtemperature=overtemperature,
            overcurrent=overcurrent,
            undervoltage=undervoltage,
            mode_status=mode_status,
            current_angle=current_angle,
            current_angular_velocity=current_angular_velocity,
            current_torque=current_torque,
            current_temperature=current_temperature
        )


@dataclass
class Type0Message(TypeMessage):
    mcu: int = 0

    def __init__(self,
                 host_id: int,
                 motor_id: int,
                 extra_info: int = 0,
                 mcu: int = 0):
        super().__init__(
            type_id=0x00,
            host_id=host_id,
            motor_id=motor_id,
            extra_info=extra_info
        )
        self.mcu = mcu

    @classmethod
    def from_decoded_fields(cls,
                            header: bytes,
                            payload: bytes) -> "Type0Message":
        """
        header = 4 bytes ya shift-revertidos por decode_message()
        payload = bytes después del header
        """
        extra_info = header[1]
        host_id = header[2]
        motor_id = header[3]

        mcu = int.from_bytes(payload, "big") if payload else 0

        return cls(
            host_id=host_id,
            motor_id=motor_id,
            extra_info=extra_info,
            mcu=mcu
        )

    def encode_message(self) -> bytes:
        header = self.build_header()
        body = b'\x01\x00'
        return self.HEADER + header + body + self.FOOTER


@dataclass
class Type1Message(TypeMessage):

    def __init__(self,
                 host_id: int,
                 motor_id: int,
                 extra_info: int = 0,
                 feed_forward: float = 0.0,
                 pos: float = 0.0,
                 vel: float = 0.0,
                 kp: float = 0.0,
                 kd: float = 0.0):
        super().__init__(
            type_id=0x01,
            host_id=host_id,
            motor_id=motor_id,
            extra_info=extra_info
        )
        self.feed_forward = map_torque_to_u16(feed_forward)
        self.pos = map_angle_to_u16(pos)
        self.vel = map_angular_vel_to_u16(vel)
        self.kp = map_Kp_to_u16(kp)
        self.kd = map_Kd_to_u16(kd)
        
    def build_header(self) -> bytes:
        # 4 bytes sin empaquetar
        raw_header = bytes([
            self.type_id & 0xFF,
            (self.feed_forward >> 8) & 0xFF,  # high byte of feed_forward
            self.feed_forward & 0xFF,          # low byte of feed_forward
            self.motor_id & 0xFF,
        ])

        # a entero
        header_val = int.from_bytes(raw_header, "big")

        # <<3 y agregar bits LSB obligatorios
        header_shifted = ((header_val << 3) | self.LSB_BITS) & 0xFFFFFFFF

        # volver a bytes
        return header_shifted.to_bytes(4, "big")

    def encode_message(self) -> bytes:
        header = self.build_header()
        raw_body = bytes([
            self.pos >> 8 & 0xFF,
            self.pos & 0xFF,
            self.vel >> 8 & 0xFF,
            self.vel & 0xFF,
            self.kp >> 8 & 0xFF,
            self.kp & 0xFF,
            self.kd >> 8 & 0xFF,
            self.kd & 0xFF
        ])
        body_val = int.from_bytes(raw_body, 'big')
        body = b'\x08' +  body_val.to_bytes(8, 'big')
        msg = self.HEADER + header + body + self.FOOTER
        return msg


@dataclass
class Type3Message(TypeMessage):

    def __init__(self,
                 host_id: int,
                 motor_id: int,
                 extra_info: int = 0):
        super().__init__(
            type_id=0x03,
            host_id=host_id,
            motor_id=motor_id,
            extra_info=extra_info
        )

    def encode_message(self) -> bytes:
        header = self.build_header()
        body = b'\x08' + b'\x00\x00\x00\x00\x00\x00\x00\x00'
        msg = self.HEADER + header + body + self.FOOTER
        return msg
    
@dataclass
class Type4Message(TypeMessage):

    def __init__(self,
                 host_id: int,
                 motor_id: int,
                 extra_info: int = 0):
        super().__init__(
            type_id=0x04,
            host_id=host_id,
            motor_id=motor_id,
            extra_info=extra_info
        )

    def encode_message(self) -> bytes:
        header = self.build_header()
        body = b'\x08' + b'\x00\x00\x00\x00\x00\x00\x00\x00'
        msg = self.HEADER + header + body + self.FOOTER
        return msg


@dataclass
class Type17Message(TypeMessage):
    index: bytes = b""
    parameter_data: bytes = b""

    def __init__(
            self, motor_id: int, host_id: int, index: bytes,
            parameter_data: bytes):
        super().__init__(type_id=0x11, host_id=host_id, motor_id=motor_id)
        self.index = index
        self.parameter_data = parameter_data

    def encode_message(self) -> bytes:
        header = self.build_header()
        body = b'\x08' + self.index + b'\x00\x00' + self.parameter_data
        return self.HEADER + header + body + self.FOOTER

    @classmethod
    def from_decoded_fields(
            cls, header: bytes, payload: bytes
    ) -> "Type17Message":
        """
        WARNING: hardcoded for index 0x7017 (see datasheet)
        """

        extra_info = header[1]
        host_id = header[2]
        motor_id = header[3]

        index = payload[1:3]
        parameter_data = payload[4:]

        return cls(
            host_id=host_id,
            motor_id=motor_id,
            index=index,
            parameter_data=parameter_data
        )


@dataclass
class Type24Message(TypeMessage):

    def __init__(
            self, motor_id: int, host_id: int):
        super().__init__(type_id=0x18, host_id=host_id, motor_id=motor_id)

    @classmethod
    def from_decoded_fields(
            cls, header: bytes, payload: bytes
    ) -> "Type24Message":
        """
        Produces a Type24Message.
        """

        extra_info = header[1]
        host_id = header[2]
        motor_id = header[3]

        report_data = payload

        return cls(
            host_id=host_id,
            motor_id=motor_id,
            report_data=report_data
        )

    def encode_message(self) -> bytes:
        header = self.build_header()
        body = b'\x08' + b'\x00\x01\x02\x03\x04\x05\x06\x01'

        msg = self.HEADER + header + body + self.FOOTER
        print("Encoded Type24Message:", msg.hex())
        # msg: "4154c007ebfc0800010203040506010d0a"
        return msg
