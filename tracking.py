import utils

import numpy as np

from typing import Tuple

from constants import *


def tracking(path: np.ndarray, curr_speed: np.ndarray, lat_speed: np.ndarray, dt: float) -> Tuple[float, float]:
    """"
    Determines the controls to follow according to the planned path.

    Args:
        path (np.ndarray): points of the path to follow.

        curr_speed (float): speed that the vehicle has at this moment.
        lat_speed (float): lateral velocity of the vehicle at this moment.

        dt (float): elabsed time between iterations.

    Return:
        Tuple[float, float]: vehicle speed and steer to follow.
    """

    steer = utils.evaluate_steer(path, lat_speed)
    steer_wheel = steer * STEERING_ANGLE_TO_STEER_WHEEL

    radius = abs(WHEELBASE / np.tan(steer))

    speed = utils.evaluate_speed(radius, curr_speed, dt)

    return steer_wheel, speed
