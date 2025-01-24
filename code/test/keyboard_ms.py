import sys
import os
import time
import msvcrt

# Make sure Python knows how to find modules in ../src
CURRENT_DIR = os.path.dirname(__file__)
SRC_DIR = os.path.join(CURRENT_DIR, "..", "src")
sys.path.append(SRC_DIR)

from unibot import Unibot
from exceptions import ODriveError

def controlLoopKeyboard(unibot):
    """
    A simple keyboard control loop for Windows, leveraging msvcrt.
    Use W/S/A/D for directional movement, F to stop, Q to quit, etc.
    """

    print("\n=== Keyboard Control ===")
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
    print("========================\n")

    speed = 0.5
    while True:
        if msvcrt.kbhit():  # Check if a key was pressed
            key = msvcrt.getch().decode('utf-8').lower()

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
                print("Invalid key, use W/A/S/D, +/-, or Q to quit")

def main():
    """
    Main entry point for testing keyboard-based control on Windows.
    """
    try:
        # Create the Unibot instance
        unibot = Unibot(max_speed=50, max_acceleration=10)

        # Set motors to closed-loop control before starting
        unibot.motor_controller.setClosedLoopControl(0)
        unibot.motor_controller.setClosedLoopControl(1)

        # Launch the keyboard control loop
        controlLoopKeyboard(unibot)

    except ODriveError as e:
        # Handle ODrive-related errors
        print("An ODrive error occurred:")
        print(e.error_struct)
    finally:
        # Deactivate motors on exit
        print("Deactivating motors...")
        try:
            unibot.motor_controller.setIdle(0)
            unibot.motor_controller.setIdle(1)
        except:
            pass

if __name__ == "__main__":
    main()
