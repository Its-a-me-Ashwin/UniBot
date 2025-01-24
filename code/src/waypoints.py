import odrive
import math
import threading
import time
import msvcrt
from odrive.enums import (
    AXIS_STATE_IDLE,
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    CONTROL_MODE_POSITION_CONTROL,
    CONTROL_MODE_VELOCITY_CONTROL
)
import argparse
import json


class Waypoint:
    """
    Represents a single waypoint or motion command for the Unibot to execute.
    
    Attributes:
      direction: (x, y) to pass to unibot.move
      speed: The speed scaling factor
      duration: How long to move in that direction before stopping
      delay: Optional delay after the move completes
    """
    def __init__(self, direction=(0, 0), speed=0.5, duration=1.0, delay=0.0):
        self.direction = direction
        self.speed = speed
        self.duration = duration
        self.delay = delay

class Waypoints:
    """
    A collection of waypoint commands. The 'execute_async' method 
    runs them one by one in a background thread.
    """
    def __init__(self):
        self.commands = []
        self._thread = None
        self._stop_flag = False

    def add_waypoint(self, waypoint: Waypoint):
        self.commands.append(waypoint)

    def execute_async(self, unibot):
        """
        Asynchronously run all waypoints on the given 'unibot'.
        Each waypoint is executed in sequence: move -> wait -> optional delay -> next.
        """
        # If a thread is already running, stop it first
        self.stop_execution(unibot)

        def worker():
            self._stop_flag = False
            for w in self.commands:
                if self._stop_flag:
                    break
                # Move as specified
                unibot.move(w.direction, w.speed)
                start_time = time.time()
                while (time.time() - start_time) < w.duration:
                    if self._stop_flag:
                        break
                    time.sleep(0.01)
                # Stop before next command
                unibot.stop()
                if w.delay > 0:
                    delay_start = time.time()
                    while (time.time() - delay_start) < w.delay:
                        if self._stop_flag:
                            break
                        time.sleep(0.01)

            unibot.stop()

        self._thread = threading.Thread(target=worker)
        self._thread.start()

    def stop_execution(self, unibot):
        """Stop any running thread and reset the robot if desired."""
        if self._thread and self._thread.is_alive():
            self._stop_flag = True
            self._thread.join()
        self._thread = None
        unibot.stop()
        print("Waypoints execution stopped.")