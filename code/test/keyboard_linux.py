import sys
import os
import time
import select
import termios
import tty

# Adjust these paths to locate your src folder if needed
CURRENT_DIR = os.path.dirname(__file__)
SRC_DIR = os.path.join(CURRENT_DIR, "..", "src")
sys.path.append(SRC_DIR)

from unibot import Unibot
from exceptione import ODriveError  # Custom exception defined in exception.py


def keyboard_loop(unibot):
    """
    Similar control loop to the Windows version, but for Linux.
    Uses select() + termios to capture single-key input without pressing ENTER.
    """

    print("\n=== Keyboard Control (Linux) ===")
    print("W: Move forward")
    print("S: Move backward")
    print("A: Turn left")
    print("D: Turn right")
    print("F: Stop motors")
    print("+: Increase speed")
    print("-: Decrease speed")
    print("M: Move axis 0 by 30 degrees (async)")
    print("N: Move axis 1 by 30 degrees (async)")
    print("R: Print encoder values")
    print("K: Kill active thread-based operations")
    print("Q: Quit and deactivate motors")
    print("================================\n")

    speed = 0.5

    # Save original terminal settings
    old_settings = termios.tcgetattr(sys.stdin.fileno())
    try:
        # Set terminal to cbreak mode (unbuffered, no echo)
        tty.setcbreak(sys.stdin.fileno())

        while True:
            # Use select to check if there's keyboard input available
            dr, dw, de = select.select([sys.stdin], [], [], 0.01)  # 10ms poll
            if dr:  # If data is ready on stdin
                key = sys.stdin.read(1).lower()  # Read one character, make it lowercase

                if key == 'w':
                    print("Moving forward")
                    unibot.move((0, 1), speed)

                elif key == 's':
                    print("Moving backward")
                    unibot.move((0, -1), speed)

                elif key == 'a':
                    print("Turning left")
                    unibot.move((-1, 0), speed)

                elif key == 'd':
                    print("Turning right")
                    unibot.move((1, 0), speed)

                elif key == 'f':
                    print("Stopping motors")
                    unibot.stop()

                elif key == 'm':
                    print("Moving axis 0 by 30 degrees (async)")
                    unibot.moveByAsync(0, 30)

                elif key == 'n':
                    print("Moving axis 1 by 30 degrees (async)")
                    unibot.moveByAsync(1, 30)

                elif key == 'r':
                    pos0, pos1 = unibot.get_encoder_values()
                    print(f"Encoder positions: Axis0={pos0:.4f}, Axis1={pos1:.4f}")

                elif key == 'k':
                    print("Killing active threaded operations...")
                    unibot.stop_current_thread()

                elif key == 'q':
                    print("Quitting and deactivating motors")
                    unibot.stop()
                    break

                elif key == '+':
                    speed += 0.5
                    print(f"Speed increased to {speed}")

                elif key == '-':
                    if speed > 0.5:
                        speed -= 0.5
                        print(f"Speed decreased to {speed}")
                    else:
                        print("Speed cannot be lower than 0.5")

                else:
                    # You may want to check for special keys like arrow keys, 
                    # but that would require additional handling.
                    print("Invalid key, use W/A/S/D, +/-, or Q to quit")

            # You can do other tasks in this loop if needed
            # ...
            time.sleep(0.01)

    finally:
        # Restore original terminal settings
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_settings)


def main():
    """
    Main entry point to demonstrate keyboard-based control on Linux.
    """
    try:
        # Instantiate Unibot
        unibot = Unibot(max_speed=50, max_acceleration=10)

        # Set both motors to closed-loop
        unibot.motor_controller.setClosedLoopControl(0)
        unibot.motor_controller.setClosedLoopControl(1)

        # Start the keyboard loop
        keyboard_loop(unibot)

    except ODriveError as e:
        print("An ODrive error occurred:")
        print(e.error_struct)

    finally:
        print("Deactivating motors...")
        try:
            unibot.motor_controller.setIdle(0)
            unibot.motor_controller.setIdle(1)
        except:
            pass


if __name__ == "__main__":
    main()
