import odrive
import math
import threading
import time
import msvcrt
import pygame
from odrive.enums import AXIS_STATE_IDLE, AXIS_STATE_CLOSED_LOOP_CONTROL, CONTROL_MODE_POSITION_CONTROL
import argparse

class ODriveMotorControl:
    """Class to control both motors of an ODrive with position control, velocity, and torque limits."""

    def __init__(self, maxVelocity, maxTorque, verbose=False):
        """Initialize ODriveMotorControl class."""
        self.odrv = self._findOdrive()
        self.odrvAxis0 = self.odrv.axis0
        self.odrvAxis1 = self.odrv.axis1
        self.verbose = verbose

        self.maxVelocity = maxVelocity
        self.maxTorque = maxTorque

        # Set the max velocity and torque for both motors
        self._setMaxVelocityTorque(self.odrvAxis0)
        self._setMaxVelocityTorque(self.odrvAxis1)

    def _findOdrive(self):
        """Connect to ODrive."""
        print("Looking for ODrive...")
        return odrive.find_any()

    def rebootOdrive(self):
        """Reboot the ODrive and reconnect to it."""
        print("Rebooting ODrive...")
        self.odrv.reboot()
        time.sleep(5)  # Give the ODrive time to reboot
        self.odrv = self._findOdrive()
        self.odrvAxis0 = self.odrv.axis0
        self.odrvAxis1 = self.odrv.axis1
        print("Reconnected to ODrive after reboot.")

    def _setMaxVelocityTorque(self, axis):
        """Set maximum velocity and torque for an axis."""
        axis.controller.config.vel_limit = self.maxVelocity
        ## TODO 
        ## Enforce the max torque
        # axis.motor.config.torque_limit = self.maxTorque

    def setIdle(self, axisNum):
        """Set the motor to idle mode (i.e., it can move freely)."""
        axis = self._getAxis(axisNum)
        axis.requested_state = AXIS_STATE_IDLE

    def setClosedLoopControl(self, axisNum):
        """Set the motor to closed-loop control."""
        axis = self._getAxis(axisNum)
        axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    def _getAxis(self, axisNum):
        """Retrieve the axis object based on axis number."""
        if axisNum == 0:
            return self.odrvAxis0
        elif axisNum == 1:
            return self.odrvAxis1
        else:
            raise ValueError("Invalid axis number. Must be 0 or 1.")

    def moveRelativePosition(self, axisNum, degrees):
        """
        Move motor by specified degrees relative to its current position, 
        respecting max velocity and torque limits.
        """
        axis = self._getAxis(axisNum)

        # Check for errors before motion
        if (self.checkErrors()):
            print("Error in Odrive. Resetting the Odrive")
            self.rebootOdrive()            
            axis = self._getAxis(axisNum)

        # Get current position in turns
        currentPosition = axis.encoder.pos_estimate

        # Calculate the target position in turns (1 turn = 360 degrees)
        targetPosition = currentPosition + (degrees / 360.0)

        # Set the control mode to position control
        axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL

        # Move the motor to the target position
        axis.controller.input_pos = targetPosition

        # Check for errors after motion
        self.checkErrors()

    def moveBothMotors(self, degrees0, degrees1):
        """
        Move both motors by the specified degrees relative to their current positions,
        non-blocking.
        """
        thread0 = threading.Thread(target=self.moveRelativePosition, args=(0, degrees0))
        thread1 = threading.Thread(target=self.moveRelativePosition, args=(1, degrees1))

        # Start both threads
        thread0.start()
        thread1.start()

        # Non-blocking, so we don't wait for threads to complete here

    def setMotorVelocity(self, axisNum, velocity):
        """Set the motor to move with a specified velocity (must be within limits)."""
        if abs(velocity) > self.maxVelocity:
            raise ValueError(f"Velocity cannot exceed {self.maxVelocity}.")

        axis = self._getAxis(axisNum)
        axis.controller.config.vel_limit = self.maxVelocity
        axis.controller.input_vel = velocity

    def setMotorTorque(self, axisNum, torque):
        """Set the motor to move with a specified torque (must be within limits)."""
        if abs(torque) > self.maxTorque:
            raise ValueError(f"Torque cannot exceed {self.maxTorque}.")

        axis = self._getAxis(axisNum)
        axis.motor.config.torque_limit = self.maxTorque
        axis.controller.input_torque = torque

    def checkErrors(self):
        """Check for errors on the ODrive and both axes and print them if any are found."""
        odrv_errors = self.odrv.error
        axis0_errors = self.odrvAxis0.error
        axis1_errors = self.odrvAxis1.error

        if odrv_errors:
            print(f"ODrive error: {odrv_errors}")
            return True
        if axis0_errors:
            print(f"Axis 0 error: {axis0_errors}")
            return True
        if axis1_errors:
            print(f"Axis 1 error: {axis1_errors}")
            return True

        if not odrv_errors and not axis0_errors and not axis1_errors:
            if self.verbose:
                print("No errors detected.")

        return False

class RobotController(object):
    def __init__(self, maxSpeed=1):
        self.wheelDiameter = 8.5 * 25.4  # mm
        self.wheelCircumference = self.wheelDiameter * math.pi

        self.motorController = ODriveMotorControl(maxSpeed, 10)

    def __del__(self):
        try:
            self.deactivateMotion()
        except AttributeError:
            pass
        except Exception as e:
            print("Unable to stop the motors.")
            ## Handle with hardware control to stop the bot.
            ## TODO:
            ## For safety this should be done before the HRI work as it is not safe.

    def deactivateMotion(self):
        self.motorController.setIdle(0)
        self.motorController.setIdle(1)

    def moveLinear(self, distance):
        requiredRotations = distance / self.wheelCircumference
        self.motorController.moveRelativePosition(0, requiredRotations * 360)
        self.motorController.moveRelativePosition(1, -requiredRotations * 360)

    def turn(self, angle):
        self.motorController.moveRelativePosition(0, angle)
        self.motorController.moveRelativePosition(1, angle)


# Keyboard Control for Windows
def controlLoopKeyboard(robot):
    """Command line interface to control the robot with wasd keys."""
    print("Control the robot with the following keys:")
    print("W: Move forward")
    print("S: Move backward")
    print("A: Turn left")
    print("D: Turn right")
    print("Q: Quit and deactivate motors")

    while True:
        if msvcrt.kbhit():  # Check if a key was pressed
            key = msvcrt.getch().decode('utf-8').lower()  # Capture the key

            if key == 'w':
                print("Moving forward")
                robot.moveLinear(100)  # Move forward by 100 mm
            elif key == 's':
                print("Moving backward")
                robot.moveLinear(-100)  # Move backward by 100 mm
            elif key == 'a':
                print("Turning left")
                robot.turn(-45)  # Turn left by 45 degrees
            elif key == 'd':
                print("Turning right")
                robot.turn(45)  # Turn right by 45 degrees
            elif key == 'q':
                print("Quitting and deactivating motors")
                robot.deactivateMotion()
                break
            else:
                print("Invalid key, use W, A, S, D to move or Q to quit")


# Xbox Controller Control with pygame
def controlLoopXbox(robot):
    """Control the robot using an Xbox controller."""
    pygame.init()
    pygame.joystick.init()

    # Initialize the first connected joystick
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    print("Xbox controller connected.")

    while True:
        # Update joystick state
        pygame.event.pump()

        # Check buttons (A = button 0, B = button 1, X = button 2, Y = button 3)
        if joystick.get_button(0):  # A button pressed
            print("Moving forward (A button pressed)")
            robot.moveLinear(100)  # Move forward by 100 mm
        elif joystick.get_button(1):  # B button pressed
            print("Moving backward (B button pressed)")
            robot.turn(45)  # Move backward by 100 mm
        elif joystick.get_button(2):  # X button pressed
            print("Turning left (X button pressed)")
            robot.turn(-45)  # Turn left by 45 degrees
        elif joystick.get_button(3):  # Y button pressed
            print("Turning right (Y button pressed)")
            robot.moveLinear(-100)  # Turn right by 45 degrees


        if joystick.get_button(6):  # Map back/select button for quitting
            print("Quitting and deactivating motors")
            robot.deactivateMotion()
            return


def controlRobot(robot, interface="keyboard"):
    """
    Starts the control loop for the robot based on the interface type.
    
    :param robot: RobotController instance
    :param interface: "keyboard" or "xbox"
    """
    if interface == "keyboard":
        controlLoopKeyboard(robot)
    elif interface == "xbox":
        controlLoopXbox(robot)
    else:
        print("Invalid interface option. Use 'keyboard' or 'xbox'.")


if __name__ == "__main__":
    robot = RobotController(maxSpeed=5)

    # Set both motors to closed-loop control
    robot.motorController.setClosedLoopControl(0)
    robot.motorController.setClosedLoopControl(1)

    parser = argparse.ArgumentParser(description="Controller")
    parser.add_argument(
        "--ct",
        type=int,
        choices=[0, 1],  # Only allow 0 or 1
        default=1,
        help="Control type",
    )
    args = parser.parse_args()
    # Choose either 'keyboard' or 'xbox' for controlling the robot
    control_type = "xbox" if args.ct == 0 else "keyboard"
    
    # Start the control loop based on the chosen interface
    controlRobot(robot, control_type)

    # Clean up and deactivate motors
    robot.deactivateMotion()
