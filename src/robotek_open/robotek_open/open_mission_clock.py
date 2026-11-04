"""
Field-equivalent clock for the end-of-round stop hold.

The WRO stop requirement (stopped and not moved for at least 30 s, Appendix A;
not moving after 15 s, Sec. 9.24.2) is elapsed time on the field. In simulation
that maps 1:1 to the ``/clock`` simulation time, not to wall time, which depends
on how fast the host runs Gazebo. Wall time is only a fallback when no ``/clock``
is published (e.g. on the real vehicle, where ROS time is real time).
"""


def mission_clock(sim_available, sim_seconds, wall_seconds):
    """
    Return ``(source, seconds)`` for the stop hold.

    ``source`` is ``'sim'`` when the ``/clock`` simulation time is being
    published (field-equivalent), otherwise ``'wall'``.
    """
    if sim_available:
        return 'sim', sim_seconds
    return 'wall', wall_seconds
