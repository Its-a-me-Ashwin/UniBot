import sys
import os
import time
import pygame

# Adjust these paths so that Python can find your src folder
CURRENT_DIR = os.path.dirname(__file__)
SRC_DIR = os.path.join(CURRENT_DIR, "..", "src")
sys.path.append(SRC_DIR)

from unibot import Unibot
from exceptions import ODriveError  # Custom ODrive exception class

def init_joystick():
    """
    Initialize pygame and the first available joystick (e.g., Xbox Controller).
    Returns the pygame joystick object.
    """
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("No joystick connected!")
        return None

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Detected joystick: {joystick.get_name()}")
    return joystick

def xbox_control_loop(unibot, joystick):
    """
    Main control loop using the Xbox controller (via pygame).
    Maps left joystick to differential movement:
      - axis 0: turn (left/right)
      - axis 1: forward/backward
    Uses buttons for additional robot functions.
    """

    # We'll define button indices for a typical Xbox controller:
    # (These may vary if you're using a different controller or mapping.)
    BTN_A = 0
    BTN_B = 1
    BTN_X = 2
    BTN_Y = 3
    BTN_LB = 4
    BTN_RB = 5
    BTN_BACK = 6
    BTN_START = 7

    # Basic instructions
    print("\n=== XBOX Controller Control ===")
    print("Left Stick:")
    print("   - Up/Down => Forward/Backward")
    print("   - Left/Right => Turn left/right")
    print("Buttons:")
    print("   A => Stop motors")
    print("   B => Move axis 0 by 30 deg (async)")
    print("   X => Move axis 1 by 30 deg (async)")
    print("   Y => Kill active threaded operations")
    print("   LB => Decrease speed factor")
    print("   RB => Increase speed factor")
    print("   BACK => Print encoder values")
    print("   START => Quit and deactivate motors")
    print("=====================================\n")

    # We can keep a "speed factor" to scale the joystick
    speed_factor = 0.5
    clock = pygame.time.Clock()

    running = True
    while running:
        # Limit loop to ~50 times per second
        clock.tick(50)

        # Pump pygame event queue
        pygame.event.pump()

        # Read joystick axes
        # Axis 0: left stick X => turning
        # Axis 1: left stick Y => forward/back
        axis_turn = joystick.get_axis(0)   # left/right
        axis_fwd = joystick.get_axis(1)   # up/down

        # Typically, pushing stick forward gives a negative axis value in pygame,
        # so we might invert axis_fwd to make forward positive:
        direction_x = axis_turn  # turn
        direction_y = -axis_fwd  # forward

        # Move the unibot with the computed direction
        # speed_factor scales the amplitude
        unibot.move((direction_x, direction_y), speed=speed_factor)

        # Read button states
        btn_a = joystick.get_button(BTN_A)
        btn_b = joystick.get_button(BTN_B)
        btn_x = joystick.get_button(BTN_X)
        btn_y = joystick.get_button(BTN_Y)
        btn_lb = joystick.get_button(BTN_LB)
        btn_rb = joystick.get_button(BTN_RB)
        btn_back = joystick.get_button(BTN_BACK)
        btn_start = joystick.get_button(BTN_START)

        # Handle buttons
        if btn_a:
            print("A pressed => Stopping motors")
            unibot.stop()

        if btn_b:
            print("B pressed => Move axis 0 by 30 deg (async)")
            unibot.moveByAsync(0, 30)

        if btn_x:
            print("X pressed => Move axis 1 by 30 deg (async)")
            unibot.moveByAsync(1, 30)

        if btn_y:
            print("Y pressed => Kill active threads")
            unibot.stop_current_thread()

        if btn_lb:
            # Decrease speed factor
            speed_factor = max(0.1, speed_factor - 0.01)
            print(f"LB pressed => Decreasing speed: {speed_factor:.2f}")

        if btn_rb:
            # Increase speed factor
            speed_factor = min(3.0, speed_factor + 0.01)
            print(f"RB pressed => Increasing speed: {speed_factor:.2f}")

        if btn_back:
            # Print encoder values
            pos0, pos1 = unibot.get_encoder_values()
            print(f"Encoder positions: Axis0={pos0:.4f}, Axis1={pos1:.4f}")

        if btn_start:
            print("START pressed => Quitting control loop")
            unibot.stop()
            running = False

def main():
    try:
        # Initialize the joystick
        joystick = init_joystick()
        if not joystick:
            print("Cannot proceed without a joystick.")
            return

        # Create the Unibot instance
        unibot = Unibot(max_speed=50, max_acceleration=10)

        # Put motors in closed-loop control
        unibot.motor_controller.setClosedLoopControl(0)
        unibot.motor_controller.setClosedLoopControl(1)

        # Start the control loop
        xbox_control_loop(unibot, joystick)

    except ODriveError as e:
        # Handle ODrive errors
        print("An ODrive error occurred:")
        print(e.error_struct)

    finally:
        # Clean up and set motors idle
        print("Deactivating motors...")
        try:
            unibot.motor_controller.setIdle(0)
            unibot.motor_controller.setIdle(1)
        except:
            pass

        # Quit pygame gracefully
        pygame.quit()

if __name__ == "__main__":
    main()
