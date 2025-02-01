import odrive
import math
import threading
import time
from odrive.enums import (
    AXIS_STATE_IDLE,
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    CONTROL_MODE_POSITION_CONTROL,
    CONTROL_MODE_VELOCITY_CONTROL
)
import argparse
import json

from waypoints import Waypoint, Waypoints
from recording import Recording
from exceptions import ODriveError
class ODriveMotorControl:
    def __init__(self, max_velocity, max_acceleration, verbose=False):
        """Initialize ODriveMotorControl class."""
        self.odrv = None
        self.odrvAxis0 = None
        self.odrvAxis1 = None
        self.verbose = verbose

        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration

        self._initialize_odrive()

    def _initialize_odrive(self):
        """Initialize and configure ODrive."""
        try:
            self.odrv = self._findOdrive()
            self.odrvAxis0 = self.odrv.axis0
            self.odrvAxis1 = self.odrv.axis1

            # Set the max velocity and ramp rate for both axes
            self._setMaxLimits(self.odrvAxis0)
            self._setMaxLimits(self.odrvAxis1)
        except Exception as e:
            print(f"Error initializing ODrive: {e}")
            self._handle_error(str(e))

    def _findOdrive(self):
        """Connect to ODrive."""
        print("Looking for ODrive...")
        return odrive.find_any()

    def _setMaxLimits(self, axis):
        """Set maximum velocity and ramp rate for an axis."""
        try:
            axis.controller.config.vel_limit = self.max_velocity
            axis.controller.config.vel_ramp_rate = self.max_acceleration
        except Exception as e:
            print(f"Error setting max limits: {e}")
            self._handle_error(str(e))

    def setAcceleration(self, acceleration):
        """Set the velocity ramp rate for both motors."""
        self.max_acceleration = acceleration
        try:
            self.odrvAxis0.controller.config.vel_ramp_rate = acceleration
            self.odrvAxis1.controller.config.vel_ramp_rate = acceleration
        except Exception as e:
            print(f"Error setting acceleration: {e}")
            self._handle_error(str(e))

    def setVelocity(self, axis_num, velocity):
        """Set the motor to move with a specified velocity."""
        if abs(velocity) > self.max_velocity:
            raise ValueError(f"Velocity cannot exceed {self.max_velocity} RPM.")

        try:
            axis = self._getAxis(axis_num)
            axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
            axis.controller.input_vel = velocity
        except Exception as e:
            print(f"Error setting velocity: {e}")
            self._handle_error(str(e))

    def moveRelativePosition(self, axis_num, degrees):
        """
        Move motor by specified degrees relative to its current position.
        """
        try:
            axis = self._getAxis(axis_num)
            current_position = axis.encoder.pos_estimate
            target_position = current_position + (degrees / 360.0)
            axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
            axis.controller.input_pos = target_position
        except Exception as e:
            print(f"Error moving relative position: {e}")
            self._handle_error(str(e))

    def setClosedLoopControl(self, axis_num):
        """Set the motor to closed-loop control mode."""
        try:
            axis = self._getAxis(axis_num)
            axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        except Exception as e:
            print(f"Error setting closed-loop control: {e}")
            self._handle_error(str(e))

    def setIdle(self, axis_num):
        """Set the motor to idle mode."""
        try:
            axis = self._getAxis(axis_num)
            axis.requested_state = AXIS_STATE_IDLE
        except Exception as e:
            print(f"Error setting idle mode: {e}")
            self._handle_error(str(e))

    def _getAxis(self, axis_num):
        """Retrieve the axis object based on axis number."""
        try:
            if axis_num == 0:
                return self.odrvAxis0
            elif axis_num == 1:
                return self.odrvAxis1
            else:
                raise ValueError("Invalid axis number. Must be 0 or 1.")
        except Exception as e:
            print(f"Error retrieving axis: {e}")
            self._handle_error(str(e))

    def _handle_error(self, message):
        """
        Handle errors by spawning a thread to reinitialize ODrive in the background.
        Raises an ODriveError to provide an error struct back to the caller.
        """
        error_struct = {
            "message": f"ODrive error occurred: {message}",
            "time": time.time(),
            "details": "Reinitialization attempt started in background thread."
        }

        def reinit_worker():
            try:
                print("Handling ODrive error in background. Resetting ODrive...")
                self.odrv.reboot()
                time.sleep(10)  # Wait for reboot
                print("Reinitializing ODrive in background thread...")
                self._initialize_odrive()
                print("Reinitialization complete.")
            except Exception as reinit_e:
                print(f"Failed to recover from error in background: {reinit_e}")

        reinit_thread = threading.Thread(target=reinit_worker, daemon=True)
        reinit_thread.start()

        # Raise an exception so caller can handle the error immediately
        raise ODriveError(error_struct)

class Unibot:
    def __init__(self, max_speed=1, max_acceleration=1):
        # For reference if needed:
        self.wheel_diameter = 8.5 * 25.4  # mm
        self.wheel_circumference = self.wheel_diameter * math.pi

        self.motor_controller = ODriveMotorControl(max_velocity=max_speed, max_acceleration=max_acceleration)

        # For thread management
        self._active_thread = None
        self._stop_thread_flag = False

    def setAcceleration(self, acceleration):
        """Set the maximum acceleration for the robot."""
        self.motor_controller.setAcceleration(acceleration)
        print(f"Max acceleration set to: {acceleration}.")

    def setVelocity(self, axis_num, velocity):
        """Set the velocity of a motor."""
        self.motor_controller.setVelocity(axis_num, velocity)

    def moveBy(self, axis_num, degrees):
        """Move the specified axis by a relative number of degrees."""
        print(f"Moving axis {axis_num} by {degrees} degrees.")
        self.motor_controller.moveRelativePosition(axis_num, degrees)

    def moveByAsync(self, axis_num, degrees):
        """
        Asynchronously move the specified axis by a relative number of degrees.
        This spawns a thread to perform the move operation and returns immediately.
        """
        def worker():
            self.moveBy(axis_num, degrees)

        self._kill_active_thread_if_any()
        self._stop_thread_flag = False
        thread = threading.Thread(target=worker)
        self._active_thread = thread
        thread.start()

    def move(self, direction=(0, 0), speed=0.5, time_duration=None):
        """
        Move the robot using an (x, y) vector.
          x > 0 => turn right
          x < 0 => turn left
          y > 0 => forward
          y < 0 => backward

        `speed` scales the movement.
        If `time_duration` is provided, it will move for that duration and then stop.
        """
        x, y = direction
        axis0_velocity = (y + x) * speed
        axis1_velocity = (-y + x) * speed

        self.setVelocity(0, axis0_velocity)
        self.setVelocity(1, axis1_velocity)

        if time_duration is not None:
            threading.Timer(time_duration, self.stop).start()

    def stop(self):
        """Stop the robot's movement."""
        print("Stopping motors")
        self.setVelocity(0, 0)
        self.setVelocity(1, 0)

    def get_encoder_values(self):
        """
        Return the current encoder position for axis0 and axis1.
        """
        pos0 = self.motor_controller.odrvAxis0.encoder.pos_estimate
        pos1 = self.motor_controller.odrvAxis1.encoder.pos_estimate
        return (pos0, pos1)

    def record(self, record_time, filename):
        """
        Put motors in idle and monitor both encoders for 'record_time' seconds.
        For each incremental change of either encoder, store the time since the last event
        and the incremental movement in a Recording object, which is then saved to 'filename'.
        """
        self.motor_controller.setIdle(0)
        self.motor_controller.setIdle(1)
        print("Motors set to IDLE. Starting recording...")

        last_pos0 = self.motor_controller.odrvAxis0.encoder.pos_estimate
        last_pos1 = self.motor_controller.odrvAxis1.encoder.pos_estimate

        recording = Recording()
        start_time = time.time()
        last_event_time = start_time

        while (time.time() - start_time) < record_time:
            current_pos0 = self.motor_controller.odrvAxis0.encoder.pos_estimate
            current_pos1 = self.motor_controller.odrvAxis1.encoder.pos_estimate

            inc0 = current_pos0 - last_pos0
            inc1 = current_pos1 - last_pos1

            if abs(inc0) > 1e-6 or abs(inc1) > 1e-6:
                now = time.time()
                delta_t = now - last_event_time
                recording.add_event(delta_t, inc0, inc1)

                last_event_time = now
                last_pos0 = current_pos0
                last_pos1 = current_pos1

            time.sleep(0.01)

        recording.save(filename)
        print(f"Recording saved to {filename}.")

    def playback(self, filename):
        """
        Read the saved recording from 'filename', set motors to closed-loop,
        then replicate each recorded event chronologically.
        """
        print(f"Loading recording from {filename}...")
        recording = Recording.load(filename)
        print("Setting motors to CLOSED-LOOP CONTROL for playback.")
        self.motor_controller.setClosedLoopControl(0)
        self.motor_controller.setClosedLoopControl(1)

        pos0 = self.motor_controller.odrvAxis0.encoder.pos_estimate
        pos1 = self.motor_controller.odrvAxis1.encoder.pos_estimate

        print("Starting playback...")
        for event in recording.events:
            time.sleep(event["delta_t"])
            pos0 += event["axis0_inc"]
            pos1 += event["axis1_inc"]

            self.motor_controller.odrvAxis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
            self.motor_controller.odrvAxis1.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
            self.motor_controller.odrvAxis0.controller.input_pos = pos0
            self.motor_controller.odrvAxis1.controller.input_pos = pos1

        print("Playback complete.")

    def _kill_active_thread_if_any(self):
        """Helper to signal the active thread to stop, if needed, and wait for it."""
        if self._active_thread and self._active_thread.is_alive():
            self._stop_thread_flag = True
            self._active_thread.join()  # Wait until it finishes
        self._active_thread = None
        self._stop_thread_flag = False

    def stop_current_thread(self):
        """
        Public method to forcefully kill any active thread-based operation
        and reset the robot to a safe state (e.g., stop the motors).
        """
        self._kill_active_thread_if_any()
        self.stop()
        print("Any active threaded operation stopped and motors halted.")

