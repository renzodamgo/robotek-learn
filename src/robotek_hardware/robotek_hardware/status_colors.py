"""RGB status code of the RRC Lite (pure logic, no ROS): what the two board RGBs show.

Simple code (4 nov 2026, the coach's request; the buzzer of the team's board is burnt out, so the LEDs are the only
signal):

  booting (no strategy state yet)      off
  ready: press the button              green
  pressed: the round runs (or ended)   blue
  battery low: change it               red blinking (over everything)

The 9 Oct code (sensor checks, calibration, finish blink) watched the camera to say whether it was alive and cost
~17 % of a CPU core next to the strategy; it was dropped.
Indexes: 1 = left RGB, 2 = right RGB (checked on the car).
"""
OFF, RED, GREEN, BLUE = (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)

READY = ('WAIT_START', 'WAIT_SENSORS')


def status_colors(state, battery_low, blink_on):
    """(left, right) colours as 0/1 triples; scale them by the brightness level.

    state: last /robotek/state ('' before the strategy publishes); blink_on: phase of the blink."""
    if battery_low:
        return (RED, RED) if blink_on else (OFF, OFF)
    if not state:
        return OFF, OFF
    if state in READY:
        return GREEN, GREEN
    return BLUE, BLUE


def scaled(color, level):
    return tuple(int(c*level) for c in color)
