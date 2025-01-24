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

class Recording:
    """
    A class to store a chronological list of incremental encoder changes.

    Each event in 'events' is a dictionary of the form:
    {
      "delta_t": <seconds since last event>,
      "axis0_inc": <incremental change for axis0>,
      "axis1_inc": <incremental change for axis1>
    }
    """
    def __init__(self):
        self.events = []

    def add_event(self, delta_t, axis0_inc, axis1_inc):
        """Add a single event to the recording."""
        self.events.append({
            "delta_t": delta_t,
            "axis0_inc": axis0_inc,
            "axis1_inc": axis1_inc
        })

    def save(self, filename):
        """Save the list of events to a JSON file."""
        with open(filename, "w") as f:
            json.dump(self.events, f, indent=2)

    @staticmethod
    def load(filename):
        """Load events from a JSON file into a Recording object."""
        with open(filename, "r") as f:
            data = json.load(f)
        rec = Recording()
        rec.events = data
        return rec