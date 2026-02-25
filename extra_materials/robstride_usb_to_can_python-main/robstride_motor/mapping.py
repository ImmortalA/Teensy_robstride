import numpy as np


def map_u16_to_angle(value: int) -> float:
    """
    Maps a 16-bit unsigned integer to an angle in radians.
    From -4π to +4π.
    """
    return ((value / 32767.0) - 1.0) * (4.0 * np.pi)

def map_angle_to_u16(value: float) -> int:
    """
    Maps an angle in radians to a 16-bit unsigned integer.
    From -4π to +4π.
    """
    if value < -4.0 * np.pi or value > 4.0 * np.pi:
        raise ValueError("Angle value out of range (-4π - 4π)")
    return int(((value + 4.0 * np.pi) / (8.0 * np.pi)) * 65535)



def map_u16_to_angular_vel(value: int) -> float:
    """
    Maps a 16-bit unsigned integer to an angular velocity in rads per second.
    From -50 rad/s to +50 rad/s.
    """
    return ((value / 32767.0) - 1.0) * 50.0

def map_angular_vel_to_u16(value: float) -> int:
    """
    Maps an angular velocity in rads per second to a 16-bit unsigned integer.
    From -50 rad/s to +50 rad/s.
    """
    if value < -50.0 or value > 50.0:
        raise ValueError("Angular velocity value out of range (-50.0 - 50.0 rad/s)")
    return int(((value + 50.0) / 100.0) * 65535)

def map_u16_to_torque(value: int) -> float:
    """
    Maps a 16-bit unsigned integer to torque in Nm.
    From -6 Nm to +6 Nm.
    """
    return ((value / 32767.0) - 1.0) * 6.0

def map_torque_to_u16(value: float) -> int:
    """
    Maps torque in Nm to a 16-bit unsigned integer.
    From -6 Nm to +6 Nm.
    """
    if value < -6.0 or value > 6.0:
        raise ValueError("Torque value out of range (-6.0 - 6.0 Nm)")
    return int(((value + 6.0) / 12.0) * 65535)


def map_u16_to_temp(value: int) -> float:
    """
    Maps a 16-bit unsigned integer to temperature in Celsius.
    OJO: Datasheet is funky, dont take this value too seriously.
    """
    return (value/10.0)

def map_Kp_to_u16(value: float) -> int:
    """
    Maps a Kp value in (0.0 - 500.0) to a 16-bit unsigned integer (0 - 65535).
    """
    if value < 0.0 or value > 500.0:
        raise ValueError("Kp value out of range (0.0 - 500.0)")
    return int((value / 500.0) * 65535)

def map_Kd_to_u16(value: float) -> int:
    """
    Maps a Kd value in (0.0 - 5.0) to a 16-bit unsigned integer (0 - 65535).
    """
    if value < 0.0 or value > 5.0:
        raise ValueError("Kd value out of range (0.0 - 5.0)")
    return int((value / 5.0) * 65535)
