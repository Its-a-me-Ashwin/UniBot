#!/usr/bin/env python3
import argparse
import sys
import time

import odrive
from odrive.enums import (AXIS_STATE_CLOSED_LOOP_CONTROL,
                          AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION,
                          AXIS_STATE_ENCODER_OFFSET_CALIBRATION,
                          AXIS_STATE_IDLE, AXIS_STATE_MOTOR_CALIBRATION,
                          CONTROL_MODE_POSITION_CONTROL, ENCODER_MODE_HALL)


class HBMotorConfig:
    HOVERBOARD_KV = 16.0
    MIN_PHASE_INDUCTANCE = 0
    MAX_PHASE_INDUCTANCE = 0.001
    MIN_PHASE_RESISTANCE = 0
    MAX_PHASE_RESISTANCE = 0.5
    ENCODER_OFFSET_FLOAT_TOLERANCE = 0.05

    def __init__(self, axis_num: int, anticogging_cal: bool, erase_config: bool) -> None:
        self.axis_num = axis_num
        self.anticogging_cal = anticogging_cal
        self.erase_config = erase_config

        print("Looking for ODrive...")
        self._find_odrive()
        print("Found ODrive.")

    def _find_odrive(self) -> None:
        self.odrv = odrive.find_any()
        self.odrv_axis = getattr(self.odrv, f"axis{self.axis_num}")

    def configure(self) -> None:
        if self.erase_config:
            print("Erasing pre-existing configuration...")
            try:
                self.odrv.erase_configuration()
            except Exception as e:
                print(f"Error erasing configuration: {e}")

        self._find_odrive()

        self.odrv.config.enable_brake_resistor = True
        self.odrv_axis.motor.config.pole_pairs = 15
        self.odrv_axis.motor.config.resistance_calib_max_voltage = 4
        self.odrv_axis.motor.config.requested_current_range = 25
        self.odrv_axis.motor.config.current_control_bandwidth = 100
        self.odrv_axis.motor.config.torque_constant = 8.27 / self.HOVERBOARD_KV
        self.odrv_axis.encoder.config.mode = ENCODER_MODE_HALL
        self.odrv_axis.encoder.config.cpr = 90
        self.odrv_axis.encoder.config.calib_scan_distance = 150
        self.odrv_axis.encoder.config.bandwidth = 100
        self.odrv_axis.controller.config.pos_gain = 6
        self.odrv_axis.controller.config.vel_gain = (
            0.02
            * self.odrv_axis.motor.config.torque_constant
            * self.odrv_axis.encoder.config.cpr
        )
        self.odrv_axis.controller.config.vel_integrator_gain = (
            0.1
            * self.odrv_axis.motor.config.torque_constant
            * self.odrv_axis.encoder.config.cpr
        )
        self.odrv_axis.controller.config.vel_limit = 10
        self.odrv_axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL

        self.odrv_axis.requested_state = AXIS_STATE_IDLE
        try:
            print("Saving manual configuration and rebooting...")
            is_saved = self.odrv.save_configuration()
            if not is_saved:
                print("Error: Configuration not saved. Are all motors in IDLE state?")
            else:
                print("Configuration saved.")
        except Exception as e:
            print(f"Error saving configuration: {e}")

        self._find_odrive()

        print("Calibrating motor...")
        self.odrv_axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION
        time.sleep(10)

        if self.odrv_axis.motor.error != 0:
            print(f"Motor calibration error: {self.odrv_axis.motor.error}")
            sys.exit(1)

        if not (self.MIN_PHASE_INDUCTANCE <= self.odrv_axis.motor.config.phase_inductance <= self.MAX_PHASE_INDUCTANCE):
            print(f"Phase inductance error: {self.odrv_axis.motor.config.phase_inductance}")
            sys.exit(1)

        if not (self.MIN_PHASE_RESISTANCE <= self.odrv_axis.motor.config.phase_resistance <= self.MAX_PHASE_RESISTANCE):
            print(f"Phase resistance error: {self.odrv_axis.motor.config.phase_resistance}")
            sys.exit(1)

        self.odrv_axis.motor.config.pre_calibrated = True

        print("Calibrating hall encoder polarity...")
        self.odrv_axis.requested_state = AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION
        time.sleep(15)

        if self.odrv_axis.encoder.error != 0:
            print(f"Encoder polarity calibration error: {self.odrv_axis.encoder.error}")
            sys.exit(1)

        print("Calibrating encoder offset...")
        self.odrv_axis.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
        time.sleep(30)

        if self.odrv_axis.encoder.error != 0:
            print(f"Encoder offset calibration error: {self.odrv_axis.encoder.error}")
            sys.exit(1)

        self.odrv_axis.encoder.config.pre_calibrated = True

        if self.anticogging_cal:
            print("Calibrating anticogging...")
            self.odrv_axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            self.odrv_axis.controller.start_anticogging_calibration()

            while self.odrv_axis.controller.config.anticogging.calib_anticogging:
                time.sleep(15)
                print("Still calibrating anticogging...")

            if self.odrv_axis.controller.error != 0:
                print(f"Anticogging calibration error: {self.odrv_axis.controller.error}")
                sys.exit(1)

            self.odrv_axis.controller.config.anticogging.pre_calibrated = True

        self.odrv_axis.requested_state = AXIS_STATE_IDLE
        try:
            print("Saving final configuration and rebooting...")
            self.odrv.save_configuration()
            print("Final configuration saved.")
        except Exception as e:
            print(f"Error saving final configuration: {e}")

        self._find_odrive()
        print("Configuration complete.")

    def mode_idle(self) -> None:
        self.odrv_axis.requested_state = AXIS_STATE_IDLE

    def mode_close_loop_control(self) -> None:
        self.odrv_axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    def move_input_pos(self, angle: float) -> None:
        self.odrv_axis.controller.input_pos = angle / 360.0


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Hoverboard Motor Calibration")
    parser.add_argument("--axis_num", type=int, choices=[0, 1], required=True, help="Motor axis number to perform calibration on (0 or 1).")
    parser.add_argument("--erase_config", action="store_true", help="Erase ODrive board's current configuration.")
    parser.add_argument("--anticogging_cal", action="store_true", help="Calibrate for anti-cogging.")
    parser.add_argument("--motor_test", action="store_true", help="Test motor movement post calibration.")

    args = parser.parse_args()

    hb_motor_config = HBMotorConfig(
        axis_num=args.axis_num,
        anticogging_cal=args.anticogging_cal,
        erase_config=args.erase_config,
    )
    hb_motor_config.configure()

    if args.motor_test:
        print("Testing motor movement in closed-loop control...")
        hb_motor_config.mode_close_loop_control()

        ## Make a 180 turn on the motor
        for angle in range(0, 210, 30):
            print(f"Moving motor to {angle} degrees.")
            hb_motor_config.move_input_pos(angle)
            time.sleep(5)

        print("Setting motor to idle mode.")
        hb_motor_config.mode_idle()
