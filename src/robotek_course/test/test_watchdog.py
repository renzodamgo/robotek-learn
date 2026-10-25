from robotek_course.watchdog import SensorWatchdog


def test_it_is_stale_until_both_sensors_have_published():
    dog = SensorWatchdog()
    assert dog.stale(0.0)
    dog.scan(0.0)
    assert dog.stale(0.0)
    dog.imu(0.0)
    assert not dog.stale(0.0)


def test_each_sensor_has_its_own_limit():
    dog = SensorWatchdog(scan_max_age=0.35, imu_max_age=0.2)
    dog.scan(1.0)
    dog.imu(1.0)
    assert not dog.stale(1.19)
    assert dog.stale(1.21)           # the gyroscope is older than 0.2 s
    dog.imu(1.21)
    assert not dog.stale(1.30)
    assert dog.stale(1.40)           # the scan is older than 0.35 s
