from robstride_motor.motor import RobstrideMotor
import numpy as np
import time
from time import sleep


def main():
    # NOTE: set `port` to your device path on Ubuntu (e.g. /dev/ttyUSB0 or /dev/ttyACM0)
    motor = RobstrideMotor(port="COM3", baudrate=921600)
    motor.set_motor_id(127)
    motor.enable_movement()

    # Sine-wave parameters (gripper open -> close)
    open_pos = 6.15
    close_pos = 7.6
    center = (open_pos + close_pos) / 2.0
    amplitude = (close_pos - open_pos) / 2.0

    # Frequency in Hz (0.2 Hz = 5 second period). Adjust to taste.
    freq = 0.2

    # Control loop rate (seconds per command). 50 Hz is a reasonable default.
    dt = 0.02
    kp = 10.0  # Proportional gain (tune for your motor)
    kd = 1.0  # Derivative gain (tune for your motor)

    print(f"Starting sine-wave motion: center={center:.3f}, amp={amplitude:.3f}, freq={freq}Hz")
    start_t = time.time()
    try:
        while True:
            t = time.time() - start_t
            pos = center + amplitude * np.cos(2 * np.pi * freq * t + np.pi)  # +pi to start at open position

            # send position command: feed_forward=0, vel=0, kp and kd tuned
            motor.operation_control(feed_forward=0.0, pos=float(pos), vel=0.0, kp=kp, kd=kd)

            # optional: lightweight status print
            if int(t) % 1 == 0:
                print(f"t={t:.2f}s target_pos={pos:.3f}", end='\r')

            time.sleep(dt)
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received — stopping and disabling movement.")
        motor.disable_movement()
    # motor.move_to_position(2.0)


if __name__ == "__main__":
    main() 
