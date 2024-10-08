import odrive
from odrive.enums import *

## Use this code to fine tune the values of the gains.
## As the weigh of the bot increases the robot needs to be retuned.

odrv0 = odrive.find_any()
change = False

### AXIS 0

current_pos_gain = odrv0.axis0.controller.config.pos_gain
current_vel_gain = odrv0.axis0.controller.config.vel_gain
current_vel_integrator_gain = odrv0.axis0.controller.config.vel_integrator_gain


## Save it somewhere as it might be needed later.
print(f"Current pos_gain: {current_pos_gain}")
print(f"Current vel_gain: {current_vel_gain}")
print(f"Current vel_integrator_gain: {current_vel_integrator_gain}")

if change:
    odrv0.axis0.controller.config.pos_gain += current_pos_gain * 0.7
    odrv0.axis0.controller.config.vel_gain += current_vel_gain * 0.7
    #odrv0.axis0.controller.config.vel_integrator_gain = current_vel_integrator_gain * 0.7


### AXIS 1


current_pos_gain = odrv0.axis1.controller.config.pos_gain
current_vel_gain = odrv0.axis1.controller.config.vel_gain
current_vel_integrator_gain = odrv0.axis1.controller.config.vel_integrator_gain


## Save it somewhere as it might be needed later.
print(f"Current pos_gain: {current_pos_gain}")
print(f"Current vel_gain: {current_vel_gain}")
print(f"Current vel_integrator_gain: {current_vel_integrator_gain}")

if change:
    odrv0.axis1.controller.config.pos_gain += current_pos_gain * 0.7
    odrv0.axis1.controller.config.vel_gain += current_vel_gain * 0.7
    #odrv0.axis1.controller.config.vel_integrator_gain = current_vel_integrator_gain * 0.7


    odrv0.save_configuration()
    #odrv0.reboot()
