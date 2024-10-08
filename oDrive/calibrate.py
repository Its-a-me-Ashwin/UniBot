"""Simplified configuration for ODrive Hoverboard motor setup."""

import argparse
import time
import odrive
from odrive.enums import (AXIS_STATE_CLOSED_LOOP_CONTROL,
                          AXIS_STATE_IDLE, AXIS_STATE_MOTOR_CALIBRATION,
                          CONTROL_MODE_POSITION_CONTROL, ENCODER_MODE_HALL)


class HBMotorConfig:
    """Class for configuring both axes of an Odrive for Hoverboard motors."""

    ## Calculated by me. So please double check if not mentioned by manufacturer.
    HOVERBOARD_KV = 16.0

    def __init__(self) -> None:
        """Initialize both axes."""
        self.odrv = self._find_odrive()
        self.odrv_axis_0 = self.odrv.axis0
        self.odrv_axis_1 = self.odrv.axis1

    def _find_odrive(self):
        # Connect to ODrive
        print("Looking for ODrive...")
        return odrive.find_any()

    def configure(self) -> None:
        """Configure both axes for hoverboard motors."""
        for axis in [self.odrv_axis_0, self.odrv_axis_1]:
            # Standard hoverboard motor configurations
            axis.motor.config.pole_pairs = 15
            axis.motor.config.resistance_calib_max_voltage = 4
            axis.motor.config.requested_current_range = 25
            ## Increase it as the motors used are quite heavy.
            axis.motor.config.current_control_bandwidth = 1500
            axis.motor.config.torque_constant = 8.27 / self.HOVERBOARD_KV
            axis.encoder.config.mode = ENCODER_MODE_HALL
            axis.encoder.config.cpr = 90
            axis.encoder.config.calib_scan_distance = 150
            axis.encoder.config.bandwidth = 100
            axis.controller.config.pos_gain = 6
            axis.controller.config.vel_gain = 0.02 * axis.motor.config.torque_constant * axis.encoder.config.cpr
            axis.controller.config.vel_integrator_gain = 0.1 * axis.motor.config.torque_constant * axis.encoder.config.cpr
            axis.controller.config.vel_limit = 10
            axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL

            # Perform motor calibration
            print(f"Calibrating motor on axis {axis}")
            axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION
            time.sleep(10)

            axis.motor.config.pre_calibrated = True
            axis.encoder.config.pre_calibrated = True

        print("Both axes configured successfully.")

    def mode_idle(self, axis) -> None:
        """Set axis to idle."""
        axis.requested_state = AXIS_STATE_IDLE

    def mode_close_loop_control(self, axis) -> None:
        """Set axis to closed loop control."""
        axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    def move_input_pos(self, axis, angle: float) -> None:
        """Move the motor on a specified axis to a certain angle."""
        axis.controller.input_pos = angle / 360.0

    def run_motor_test(self, motor_test: bool) -> None:
        """Run motor tests on both axes if enabled."""
        if motor_test:
            for axis_num, axis in enumerate([self.odrv_axis_0, self.odrv_axis_1]):
                print(f"Placing motor {axis_num} in closed-loop control.")
                self.mode_close_loop_control(axis)

                print(f"CONDUCTING MOTOR TEST for motor {axis_num}")
                for angle in range(0, 390, 30):
                    print(f"Setting motor {axis_num} to {angle} degrees.")
                    self.move_input_pos(axis, angle)
                    time.sleep(5)

                print(f"Placing motor {axis_num} in idle.")
                self.mode_idle(axis)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Hoverboard Motor Configuration")

    # Argument for conducting motor tests
    parser.add_argument(
        "--motor_test",
        action="store_true",
        help="Test conducted at end of calibration to move motors in 30 deg increments.",
    )

    args = parser.parse_args()

    # Initialize and configure both axes
    hb_motor_config = HBMotorConfig()
    hb_motor_config.configure()

    # Run motor tests if the flag is provided
    hb_motor_config.run_motor_test(motor_test=args.motor_test)
