Software Support
================

Getting Started
---------------

#. **Set up a Raspberry Pi**:
   - If you use a Raspberry Pi or a similar single-board computer, please set it up with an OS.

   - It is recommended to update to the latest version of the OS, set up a WiFi connection, and enable SSH.
   
   - Also, set up Bluetooth and connect a controller to the Raspberry Pi for debugging later. Refer to :doc:`bluetooth_setup` for instructions.

#. **Install Dependencies**:
   - Install Python and the required packages.

   .. code-block:: bash

      sudo apt update
      sudo apt-get install python3 python3-pip

      pip3 install odrive

      cd ~
      git clone https://github.com/<removed-for-anonymity>/Unibot.git
      cd Unibot/code
      pip install -e .

Initial Calibration
-------------------

Run the following command while the robot is on a stand with its wheels free to move. Ensure that the ODrive has power before proceeding.

#. **Configure the ODrive**:

   .. code-block:: bash

      sudo ./Unibot/code/src/config.sh

   This command takes about one minute to run, and the robot's wheels will move during the process. Once it completes, **do not** change the orientation of the wires.

   At this point, your robot is ready to use.

Example Usage of the Repository
-------------------------------

Run the folloing to set up a simple waypoint mission to make the robot move around.

.. code-block:: bash

   #!/usr/bin/env python3
   import time
   from unibot import Unibot
   from unibot import Waypoints, Waypoint  # Or "from unibot.waypoints import Waypoints, Waypoint" if you prefer

   def main():
      # 1. Create your Unibot instance
      unibot = Unibot(max_speed=50, max_acceleration=10)

      # 2. Set the motors to closed-loop control
      unibot.motor_controller.setClosedLoopControl(0)
      unibot.motor_controller.setClosedLoopControl(1)
      print("Motors set to CLOSED-LOOP control.")

      # 3. Define some waypoints
      #    Each Waypoint has a direction (x, y), a speed, a duration, and an optional delay after the move
      waypoints = Waypoints()
      waypoints.add_waypoint(Waypoint(direction=(0, 1), speed=0.5, duration=3.0, delay=1.0))  # Move forward
      waypoints.add_waypoint(Waypoint(direction=(1, 0), speed=0.3, duration=2.0, delay=1.0))  # Turn right
      waypoints.add_waypoint(Waypoint(direction=(0, -1), speed=0.5, duration=3.0, delay=0.5)) # Move backward
      waypoints.add_waypoint(Waypoint(direction=(-1, 0), speed=0.3, duration=2.0, delay=0.0)) # Turn left

      # 4. Execute the waypoints asynchronously
      print("Executing waypoints...")
      waypoints.execute_async(unibot)

      # (Optional) Let the robot do its moves for 10 seconds
      time.sleep(10)

      # 5. Stop execution (if you want to halt prematurely or after finishing)
      print("Stopping waypoints execution...")
      waypoints.stop_execution(unibot)

      # Deactivate the motors or keep them in closed-loop as you see fit
      unibot.motor_controller.setIdle(0)
      unibot.motor_controller.setIdle(1)
      print("Motors set to IDLE. Done.")

   if __name__ == "__main__":
      main()
