"""Placeholder gait timing definitions."""

LEG_NAMES = ['front_left', 'front_right', 'rear_left', 'rear_right']

_TROT_OFFSETS = {
    'front_left': 0.0,
    'rear_right': 0.0,
    'front_right': 0.5,
    'rear_left': 0.5,
}

_CRAWL_OFFSETS = {
    'front_left': 0.0,
    'rear_right': 0.25,
    'front_right': 0.50,
    'rear_left': 0.75,
}


def phase_offset_for_gait(gait_name: str, leg_name: str) -> float:
    """Return normalized phase offset for a named gait and leg."""
    if gait_name == 'crawl':
        return _CRAWL_OFFSETS.get(leg_name, 0.0)
    return _TROT_OFFSETS.get(leg_name, 0.0)


def duty_factor_for_gait(gait_name: str) -> float:
    """Return stance fraction for the gait cycle."""
    if gait_name == 'crawl':
        return 0.75
    return 0.5
