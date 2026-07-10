"""Shared controller defaults."""

LEG_NAMES = ['front_left', 'front_right', 'rear_left', 'rear_right']
JOINT_SUFFIXES = ['hip_abduction_joint', 'hip_pitch_joint', 'knee_pitch_joint']
DEFAULT_JOINT_NAMES = [
    f'{leg}_{suffix}'
    for leg in LEG_NAMES
    for suffix in JOINT_SUFFIXES
]
