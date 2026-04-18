import math

from ugv_bringup.goal_pose_service import resolve_goal_from_absolute_xy_yaw


def test_resolve_goal_from_absolute_xy_yaw_normalizes_heading():
    goal_x, goal_y, goal_yaw = resolve_goal_from_absolute_xy_yaw(
        goal_x=1.25,
        goal_y=-0.75,
        goal_yaw=3.0 * math.pi,
        normalize_goal_yaw=True,
    )

    assert math.isclose(goal_x, 1.25, abs_tol=1.0e-6)
    assert math.isclose(goal_y, -0.75, abs_tol=1.0e-6)
    assert math.isclose(goal_yaw, math.pi, abs_tol=1.0e-6)


def test_resolve_goal_from_absolute_xy_yaw_can_keep_original_heading():
    goal_x, goal_y, goal_yaw = resolve_goal_from_absolute_xy_yaw(
        goal_x=-2.0,
        goal_y=0.5,
        goal_yaw=4.0 * math.pi,
        normalize_goal_yaw=False,
    )

    assert math.isclose(goal_x, -2.0, abs_tol=1.0e-6)
    assert math.isclose(goal_y, 0.5, abs_tol=1.0e-6)
    assert math.isclose(goal_yaw, 4.0 * math.pi, abs_tol=1.0e-6)
