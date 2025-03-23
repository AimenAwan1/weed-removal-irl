import numpy as np


def get_angle_quadrant(th):
    if th >= 0:
        if th <= np.pi/2:
            return 1
        else:
            return 2
    else:
        if th >= -np.pi/2:
            return 4
        else:
            return 3


def compute_ang_from_horiz(th):
    if -np.pi / 2 <= th <= np.pi / 2:
        return np.abs(th)
    elif th > np.pi / 2:
        return np.pi - th
    else:
        return th - (-np.pi)


def compute_angle_diff(th1, th2, wrap=False):
    if not wrap:
        return th2 - th1
    else:
        # makes the angles continuous between quadrants 3 and 4
        th1_continuous = (th1 + 2 * np.pi) % (2 * np.pi)
        th2_continuous = (th2 + 2 * np.pi) % (2 * np.pi)
        return th2_continuous - th1_continuous


def compute_err_angle(th1, th2):
    th1_quadrant = get_angle_quadrant(th1)
    th2_quadrant = get_angle_quadrant(th2)

    # no concerns with angle wraparound
    same_quadrant = th1_quadrant == th2_quadrant

    # finds that when passing between both angles it does not pass the
    # wraparound point between quadrants 3 and 4
    min_quadrant = min(th1_quadrant, th2_quadrant)
    max_quadrant = max(th1_quadrant, th2_quadrant)

    quadrants_are_continuous = (
        (min_quadrant == 1 and max_quadrant == 2)
        or (max_quadrant == 4 and min_quadrant == 1)
        or (min_quadrant == 3 and max_quadrant == 4)
    )

    if same_quadrant or quadrants_are_continuous:
        return compute_angle_diff(th1, th2, wrap=False)
    elif min_quadrant == 2 and max_quadrant == 3:
        return compute_angle_diff(th1, th2, wrap=True)

    # only remaining options are where the angles are in opposite quadrants
    th1_ang_from_horiz = compute_ang_from_horiz(th1)
    th2_ang_from_horiz = compute_ang_from_horiz(th2)

    if th2_quadrant == 3 or th2_quadrant == 2:
        compute_with_wraparound = th2_ang_from_horiz <= th1_ang_from_horiz
    else:
        compute_with_wraparound = th2_ang_from_horiz >= th1_ang_from_horiz
    return compute_angle_diff(th1, th2, compute_with_wraparound)
