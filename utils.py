import numpy as np

from typing import Tuple

from constants import *


controller = None


class PID:
    def __init__(self, kp: float, ki: float, kd: float) -> None:
        """
        Called when PID object is initialized.

        Args:
            kp (float): P parameter of PID.
            ki (float): I parameter of PID.
            kd (float): D parameter of PID.
        """

        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.prev_error = 0.0

        self.integrator = 0.0

    def evaluate_correction(self, curr: float, target: float, dt: float) -> float:
        """
        Evaluate the correction using the PID controller.

        Args:
            curr (float): current value.
            target (float): target value.

            dt (float): elapsed time between iterations.

        Returns:
            float: correction increment.
        """

        error = target - curr

        self.integrator += error * dt
        derivative = (error - self.prev_error) / dt

        self.prev_error = error

        return (
            error * self.kp + self.integrator * self.ki + derivative * self.kd
        )


def get_closest_point(points: np.ndarray, origin: np.ndarray) -> int:
    """
    Get the closest point in relation to a reference point.

    Args:
        points (np.ndarray): set of points to get the closest.
        origin (np.ndarray): reference point to get the closest.

    Returns:
        int: respective index of closest point.
    """

    vectors = points - origin

    squared_norms = np.einsum("ij,ij->i", vectors, vectors)
    index = np.argmin(squared_norms)

    return index


def evaluate_steer(path: np.ndarray, lat_speed: float) -> float:
    """
    Evaluate the steering angle according to the Stanley method.

    Args:
        path (np.ndarray): set of points that form the path of the car.
        lat_speed (float): velocity of the car its lateral direction.

    Returns:
        float: the front wheel steer angle for the curvature of the track.
    """

    index = get_closest_point(path, FRONT_AXLE)

    if index == np.shape(path)[0] - 1:
        index -= 1

    diff = path[index+1] - path[index]
    psi = np.arctan2(diff[1], diff[0])

    cross_track_error = np.linalg.norm(path[index] - FRONT_AXLE)

    steer = psi + np.arctan(
        (STANLEY_GAIN * cross_track_error) / (lat_speed + STANLEY_DEN)
    )

    steer = np.clip(
        steer, MIN_STEERING_ANGLE, MAX_STEERING_ANGLE
    )

    return steer


def evaluate_speed(radius: float, curr_speed: float, dt: float) -> float:
    """
    Evaluate the speed according to the PID method.

    Args:
        radius (float): path curvature radius.
        curr_speed (float): the current speed of the car.

        dt (float): elapsed time between iterations.

    Returns:
        float: the required speed for the curvature of the track.
    """

    global controller

    if controller is None:
        controller = PID(SPEED_KP, SPEED_KI, SPEED_KD)

    target_speed = np.clip(
        np.sqrt(MAX_LATERAL_ACC * radius), MIN_SPEED, MAX_SPEED
    )

    correction = controller.evaluate_correction(
        curr_speed, target_speed, dt
    )

    speed = np.clip(
        curr_speed + correction, MIN_SPEED, MAX_SPEED
    )

    return speed

