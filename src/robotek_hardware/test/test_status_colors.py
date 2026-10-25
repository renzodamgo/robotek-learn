from robotek_hardware.status_colors import (BLUE, GREEN, MAGENTA, OFF, RED, YELLOW, scaled,
                                            status_colors)


def colors(state='WAIT_START', lidar=True, imu=True, camera=True, low=False, booting=False, blink=True):
    return status_colors(state, lidar, imu, camera, low, booting, blink)


def test_waiting_for_start():
    assert colors() == (GREEN, GREEN)
    assert colors(state='') == (GREEN, GREEN)              # strategy not publishing yet
    assert colors(state='WAIT_SENSORS') == (GREEN, GREEN)  # Obstacle before Start


def test_missing_sensor_before_start():
    assert colors(lidar=False) == (RED, BLUE)
    assert colors(imu=False) == (RED, MAGENTA)
    assert colors(camera=False) == (RED, YELLOW)
    assert colors(lidar=False, camera=False) == (RED, BLUE)   # LiDAR first
    assert colors(camera=False, booting=True) == (OFF, OFF)   # drivers still starting
    assert colors(booting=True) == (GREEN, GREEN)


def test_round_states():
    assert colors(state='CALIBRATE') == (BLUE, BLUE)
    for state in ('DETECT_DIRECTION', 'FOLLOW', 'TURN', 'FINISH', 'OPERATOR_STOP', 'WAIT_SENSOR'):
        assert colors(state=state, camera=False) == (OFF, OFF)
    assert colors(state='DONE') == (GREEN, GREEN) and colors(state='DONE', blink=False) == (OFF, OFF)
    assert colors(state='PARKED', blink=False) == (OFF, OFF)
    for state in ('FAULT', 'DIRECTION_FAULT', 'STALLED'):
        assert colors(state=state) == (RED, RED)


def test_low_battery_wins():
    assert colors(state='FOLLOW', low=True) == (RED, RED)
    assert colors(state='DONE', low=True, blink=False) == (OFF, OFF)
    assert colors(state='WAIT_START', lidar=False, low=True) == (RED, RED)


def test_scaled():
    assert scaled(YELLOW, 40) == (40, 40, 0)
