import math

from ugv_bringup.relative_goal_pose_service import resolve_goal_from_relative_xy


def test_resolve_goal_from_relative_xy_keeps_heading_by_default():
    goal_x, goal_y, goal_yaw = resolve_goal_from_relative_xy(
        current_x=1.0,
        current_y=2.0,
        current_yaw=math.pi / 2.0,
        relative_x=1.0,
        relative_y=0.0,
        align_yaw_to_motion=False,
    )

    assert math.isclose(goal_x, 1.0, abs_tol=1.0e-6)
    assert math.isclose(goal_y, 3.0, abs_tol=1.0e-6)
    assert math.isclose(goal_yaw, math.pi / 2.0, abs_tol=1.0e-6)


def test_resolve_goal_from_relative_xy_can_align_heading_to_motion():
    goal_x, goal_y, goal_yaw = resolve_goal_from_relative_xy(
        current_x=0.0,
        current_y=0.0,
        current_yaw=math.pi / 4.0,
        relative_x=0.0,
        relative_y=1.0,
        align_yaw_to_motion=True,
    )

    assert math.isclose(goal_x, -math.sqrt(0.5), abs_tol=1.0e-6)
    assert math.isclose(goal_y, math.sqrt(0.5), abs_tol=1.0e-6)
    assert math.isclose(goal_yaw, 3.0 * math.pi / 4.0, abs_tol=1.0e-6)
