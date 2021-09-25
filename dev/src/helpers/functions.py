from math import cos, sin

def feet_to_meters(feet) -> float:
    '''
        Converts feet to meters.

        :param feet
    '''
    return feet * 0.3048

def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> dict:
    '''
        Converts euler coordinates to quaternion coordinates.
        Assumes all values given are in radians.

        :param roll: X coordinate
        :param pitch: Y coordinate
        :param yaw: Z coordinate
    '''
    # Abbreviations for the various angular functions
    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)

    return {
        'x': sr * cp * cy - cr * sp * sy,
        'y': cr * sp * cy + sr * cp * sy,
        'z': cr * cp * sy - sr * sp * cy,
        'w': cr * cp * cy + sr * sp * sy
    }