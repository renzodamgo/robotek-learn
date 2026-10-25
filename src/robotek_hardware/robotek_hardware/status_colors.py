"""RGB status code of the RRC Lite (pure logic, no ROS): what the two board RGBs show.

Agreed with the team on 9 Oct 2026 (the buzzer of the team's board is burnt out, so the LEDs are the only signal):

  booting, no sensor data yet          off             off
  ready, waiting for Start             green           green
  a sensor is missing before Start     red             blue = LiDAR, magenta = IMU, yellow = camera
  calibrating after Start              blue            blue
  running the round                    off             off
  finished (DONE / PARKED)             green blinking  green blinking
  battery low: change it               red blinking    red blinking

Not in the table, added: a strategy fault (direction unresolved, stalled, Obstacle FAULT) is red
on both; the stop by the button (OPERATOR_STOP) is off like a running round.
Indexes: 1 = left RGB, 2 = right RGB (checked on the car).
"""
OFF, RED, GREEN, BLUE, YELLOW, MAGENTA = (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1), (1, 1, 0), (1, 0, 1)

BEFORE_START = ('', 'WAIT_START', 'WAIT_SENSORS')
CALIBRATING = ('CALIBRATE',)
FINISHED = ('DONE', 'PARKED')
FAULTS = ('FAULT', 'DIRECTION_FAULT', 'STALLED', 'PARK_FAIL')


def status_colors(state, lidar_ok, imu_ok, camera_ok, battery_low, booting, blink_on):
    """(left, right) colours as 0/1 triples; scale them by the brightness level.

    state: last /robotek/state ('' before the strategy publishes); *_ok: fresh data;
    booting: still inside the grace time after launch; blink_on: phase of the blink."""
    if battery_low:
        return (RED, RED) if blink_on else (OFF, OFF)
    if state in FINISHED:
        return (GREEN, GREEN) if blink_on else (OFF, OFF)
    if state in FAULTS:
        return RED, RED
    if state in CALIBRATING:
        return BLUE, BLUE
    if state not in BEFORE_START:
        return OFF, OFF            # running (or stopped by the button): do not distract
    if lidar_ok and imu_ok and camera_ok:
        return GREEN, GREEN
    if booting:
        return OFF, OFF
    missing = BLUE if not lidar_ok else MAGENTA if not imu_ok else YELLOW
    return RED, missing


def scaled(color, level):
    return tuple(int(c*level) for c in color)
