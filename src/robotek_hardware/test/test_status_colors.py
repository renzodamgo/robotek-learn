from robotek_hardware.status_colors import BLUE, GREEN, OFF, RED, scaled, status_colors


def test_off_while_booting_green_blinking_when_ready_blue_once_pressed():
    assert status_colors('', False, True) == (OFF, OFF)
    assert status_colors('WAIT_START', False, True) == (GREEN, GREEN)
    assert status_colors('WAIT_START', False, False) == (OFF, OFF)
    for state in ('CALIBRATE', 'DETECT_DIRECTION', 'FOLLOW', 'TURN', 'FINISH', 'DONE', 'OPERATOR_STOP'):
        assert status_colors(state, False, True) == (BLUE, BLUE)


def test_low_battery_blinks_red_over_everything():
    assert status_colors('FOLLOW', True, True) == (RED, RED)
    assert status_colors('WAIT_START', True, False) == (OFF, OFF)


def test_scaled():
    assert scaled(GREEN, 2) == (0, 2, 0)
