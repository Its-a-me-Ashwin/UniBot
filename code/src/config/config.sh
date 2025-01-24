#!/bin/bash

## Run this script to configure the odrive for two sakte board motors.
python3 ./odriveConfig.py --axis_num=0 --erase_config --anticogging_cal
python3 ./odriveConfig.py --axis_num=1 --anticogging_cal