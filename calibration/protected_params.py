"""
calibration/protected_params.py

Parameters set by QGroundControl calibration that Python must NEVER write.

These are set once during physical calibration (6-position accel, compass)
and must survive all Python param pushes unchanged.

Any param manager or script that writes parameters must filter these out.

Usage:
    from calibration.protected_params import PROTECTED, is_protected

    if not is_protected(param_name):
        set_param(master, param_name, value)
"""

# Accelerometer offsets and scale factors (set by 6-position accel cal)
_ACCEL = [
    'INS_ACCOFFS_X', 'INS_ACCOFFS_Y', 'INS_ACCOFFS_Z',
    'INS_ACCSCAL_X', 'INS_ACCSCAL_Y', 'INS_ACCSCAL_Z',
    'INS_ACC2OFFS_X', 'INS_ACC2OFFS_Y', 'INS_ACC2OFFS_Z',
    'INS_ACC2SCAL_X', 'INS_ACC2SCAL_Y', 'INS_ACC2SCAL_Z',
    'INS_ACC3OFFS_X', 'INS_ACC3OFFS_Y', 'INS_ACC3OFFS_Z',
    'INS_ACC3SCAL_X', 'INS_ACC3SCAL_Y', 'INS_ACC3SCAL_Z',
]

# Gyro offsets (set by gyro cal on boot and calibrate.py)
_GYRO = [
    'INS_GYROFFS_X', 'INS_GYROFFS_Y', 'INS_GYROFFS_Z',
    'INS_GYR2OFFS_X', 'INS_GYR2OFFS_Y', 'INS_GYR2OFFS_Z',
    'INS_GYR3OFFS_X', 'INS_GYR3OFFS_Y', 'INS_GYR3OFFS_Z',
]

# Compass calibration (set by compass cal in QGC)
_COMPASS = [
    'COMPASS_OFS_X',  'COMPASS_OFS_Y',  'COMPASS_OFS_Z',
    'COMPASS_OFS2_X', 'COMPASS_OFS2_Y', 'COMPASS_OFS2_Z',
    'COMPASS_OFS3_X', 'COMPASS_OFS3_Y', 'COMPASS_OFS3_Z',
    'COMPASS_DIA_X',  'COMPASS_DIA_Y',  'COMPASS_DIA_Z',
    'COMPASS_ODI_X',  'COMPASS_ODI_Y',  'COMPASS_ODI_Z',
    'COMPASS_DIA2_X', 'COMPASS_DIA2_Y', 'COMPASS_DIA2_Z',
    'COMPASS_ODI2_X', 'COMPASS_ODI2_Y', 'COMPASS_ODI2_Z',
]

# Board orientation and trim (set by calibrate.py — must stay at 29/0)
_AHRS = [
    'AHRS_ORIENTATION',
    'AHRS_TRIM_X',
    'AHRS_TRIM_Y',
    'AHRS_TRIM_Z',
]

PROTECTED: frozenset = frozenset(_ACCEL + _GYRO + _COMPASS + _AHRS)


def is_protected(param_name: str) -> bool:
    """Return True if this parameter must not be written by Python."""
    return param_name in PROTECTED
