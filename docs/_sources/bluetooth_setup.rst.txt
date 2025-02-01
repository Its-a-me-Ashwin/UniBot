Connecting Xbox Controller to Raspberry Pi via SSH
==================================================

This guide provides step-by-step instructions to set up and connect an Xbox controller to a Raspberry Pi over SSH, using only the terminal.

.. contents:: Table of Contents
   :depth: 2
   :local:

Setting up Dependencies
-----------------------

To enable Bluetooth and install necessary drivers:

1. **Enable and Start Bluetooth Service** ::

    sudo systemctl enable bluetooth
    sudo systemctl start bluetooth

2. **Update the Package List** ::

    sudo apt update

3. **Install Xbox Driver** ::

    sudo apt install xboxdrv

Connecting the Xbox Controller
------------------------------

To connect the Xbox controller, follow these steps using ``bluetoothctl``:

1. **Open the Bluetooth Controller** ::

    bluetoothctl

2. **Turn Bluetooth Power On** ::

    power on

3. **Enable Agent** ::

    agent on

4. **Set the Default Agent** ::

    default-agent

5. **Start Scanning for Devices** ::

    scan on

   - Look for the MAC address and name of your Xbox controller in the scan results.

6. **Pair the Controller** ::

    pair <MAC Address>

7. **Trust the Controller** ::

    trust <MAC Address>

8. **Connect the Controller** ::

    connect <MAC Address>

9. **Exit Bluetooth Controller** ::

    exit

   - The controller should now be connected to your Raspberry Pi.

Setting up Python Environment
-----------------------------

For this project, a specific Python environment is required:

1. **Run Python from the Virtual Environment** ::

    sudo ~/Projects/venv/bin/python

2. **(Optional) Create an Alias for Convenience** 

   - To simplify running the specific Python version, create an alias by adding the following line to your ``~/.bashrc`` or ``~/.bash_aliases`` file: ::

       alias mypython='sudo ~/Projects/venv/bin/python'

   - After adding the alias, reload your shell: ::

       source ~/.bashrc

   - You can now use ``mypython`` to run the project-specific Python.

---

This completes the setup for connecting an Xbox controller to your Raspberry Pi via SSH and configuring the required Python environment.
