"""Planar two-link arm kinematics for the drawbot.

Angles are in radians, lengths in metres. The shoulder sits at the origin of
the base frame, x points out over the paper and y to the left.
"""

import math

# Must match urdf/drawbot.urdf.xacro
L1 = 0.14            # shoulder axis -> elbow axis
L2 = 0.14            # elbow axis -> pen tip (pen down)
SHOULDER_LIMIT = 2.6
ELBOW_LIMIT = 2.7

# A5 sheet, landscape, as placed on the board (base frame)
PAPER_X = (0.090, 0.238)
PAPER_Y = (-0.105, 0.105)
PAPER_CENTER = (0.164, 0.0)

# NEMA 17: 200 full steps per rev
STEPS_PER_REV = 200


def forward(shoulder, elbow, l1=L1, l2=L2):
    """Pen tip (x, y) for the given joint angles."""
    x = l1 * math.cos(shoulder) + l2 * math.cos(shoulder + elbow)
    y = l1 * math.sin(shoulder) + l2 * math.sin(shoulder + elbow)
    return x, y


def inverse(x, y, elbow_up=False, l1=L1, l2=L2):
    """Joint angles (shoulder, elbow) reaching (x, y), or None if unreachable.

    elbow_up=False gives a negative elbow angle (elbow bends to the left of
    the shoulder->pen line), which keeps the arm clear of the paper's centre.
    """
    d2 = x * x + y * y
    c = (d2 - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
    if c < -1.0 or c > 1.0:
        return None
    elbow = math.acos(c)
    if not elbow_up:
        elbow = -elbow
    shoulder = math.atan2(y, x) - math.atan2(l2 * math.sin(elbow), l1 + l2 * math.cos(elbow))
    shoulder = math.atan2(math.sin(shoulder), math.cos(shoulder))
    if abs(shoulder) > SHOULDER_LIMIT or abs(elbow) > ELBOW_LIMIT:
        return None
    return shoulder, elbow


def quantize(angle, microsteps=16):
    """Snap an angle to the nearest stepper microstep."""
    if microsteps <= 0:
        return angle
    step = 2.0 * math.pi / (STEPS_PER_REV * microsteps)
    return round(angle / step) * step
