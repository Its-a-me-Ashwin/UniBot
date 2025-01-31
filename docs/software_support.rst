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

Run the following commands to start your robot’s software.

.. code-block:: bash

   cd MyRobotPupper
   python3 main.py

Troubleshooting
---------------

- **Issue 1**: Symptom, cause, and solution.
- **Issue 2**: Symptom, cause, and solution.
