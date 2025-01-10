
import math
import numpy as np
from rsoccer_gym.Entities import Robot
from utils.Point import Point
from utils.Geometry import Geometry
from utils.ssl.Navigation import Navigation


PROP_VELOCITY_MIN_FACTOR: float = 0.1
MAX_VELOCITY: float = 1.5
ANGLE_EPSILON: float = 0.1
ANGLE_KP: float = 5
MIN_DIST_TO_PROP_VELOCITY: float = 720

ADJUST_ANGLE_MIN_DIST: float = 50
M_TO_MM: float = 1000.0


class AgentNavigation:

  @staticmethod
  def goToPoint(robot: Robot, target: Point):
    target = Point(target.x * M_TO_MM, target.y * M_TO_MM)
    robot_position = Point(robot.x * M_TO_MM, robot.y * M_TO_MM)
    robot_angle = Navigation.degrees_to_radians(Geometry.normalize_angle(robot.theta, 0, 180))

    max_velocity = MAX_VELOCITY
    distance_to_target = robot_position.dist_to(target)
    kp = ANGLE_KP

    # Use proportional speed to decelerate when getting close to desired target
    proportional_velocity_factor = PROP_VELOCITY_MIN_FACTOR
    min_proportional_distance = MIN_DIST_TO_PROP_VELOCITY

    if distance_to_target <= min_proportional_distance:
      max_velocity = max_velocity * Navigation.map_value(distance_to_target, 0.0, min_proportional_distance, proportional_velocity_factor, 1.0)

    target_angle = (target - robot_position).angle()
    d_theta = Geometry.smallest_angle_diff(target_angle, robot_angle)

    if distance_to_target > ADJUST_ANGLE_MIN_DIST:
      v_angle = Geometry.abs_smallest_angle_diff(math.pi - ANGLE_EPSILON, d_theta)

      v_proportional = v_angle * (max_velocity / (math.pi - ANGLE_EPSILON))
      global_final_velocity = Geometry.from_polar(v_proportional, target_angle)
      target_velocity = Navigation.global_to_local_velocity(global_final_velocity.x, global_final_velocity.y, robot_angle)

      return target_velocity, -kp * d_theta
    else:
      return Point(0.0, 0.0), -kp * d_theta