
import math
import numpy as np
from rsoccer_gym.Entities import Robot
from utils.Point import Point
from utils.Geometry import Geometry
from utils.ssl.Navigation import Navigation
from typing import Tuple


PROP_VELOCITY_MIN_FACTOR: float = 0.1
MAX_VELOCITY: float = 1.5
ANGLE_EPSILON: float = 0.1
ANGLE_KP: float = 5
MIN_DIST_TO_PROP_VELOCITY: float = 720

ADJUST_ANGLE_MIN_DIST: float = 50
M_TO_MM: float = 1000.0

MIN_DISTANCE_TO_TURN = 220

class AgentNavigation:

  @staticmethod
  def min_dist(robot : Robot, opponents: dict[int, Robot] = dict()) -> Tuple[float, float]:
    robot_position = Point(robot.x * M_TO_MM, robot.y * M_TO_MM)
    m_dist = 10000
    closest_opponent = Point(0, 0)
    for i in range(1, 22):
      if(opponents.get(i) is not None):
        opponent = opponents[i]
        opponent_position = Point(opponent.x * M_TO_MM, opponent.y * M_TO_MM)
        dist = robot_position.dist_to(opponent_position)
        if(dist < m_dist):
          m_dist = dist
          closest_opponent = opponent_position
    return m_dist, closest_opponent
  
  @staticmethod
  def rotate(vector : Point, angle : float) -> Point:          #Sentido Anti-Horário
    x = vector.x*math.cos(angle)-vector.y*math.sin(angle)
    y = vector.x*math.sin(angle) + vector.y*math.cos(angle)
    return Point(x, y)
  
  @staticmethod
  def avoid_obstacle(dist_center : Point, angle_center : float):
    if(angle_center > 0):
      return AgentNavigation.rotate(dist_center, -math.pi/2)
    else:
      return AgentNavigation.rotate(dist_center, math.pi/2)

  
  @staticmethod
  def new_direction(robot: Robot, target: Point, obstacle: Point):
    robot_position = Point(robot.x * M_TO_MM, robot.y * M_TO_MM)

    obs_to_goal = Point(target.x, target.y).__sub__(obstacle)
    robot_to_obs = Point(obstacle.x, obstacle.y).__sub__(robot_position)
    angle_to_turn = robot_to_obs.angle() - obs_to_goal.angle()

    print(obs_to_goal)
    print(robot_to_obs)
    print(angle_to_turn)

    angle_center = Geometry.normalize_angle(angle_to_turn)
    new_dir = AgentNavigation.avoid_obstacle(robot_to_obs, angle_center)

    print(new_dir)

    return new_dir
  
  @staticmethod
  def total_velocity(new_dir : Point, dir : Point, min_dist):
    coef = min_dist**2/(MIN_DISTANCE_TO_TURN**2)
    new_coef = 1 - coef
    new_dir.__mul__(new_coef)
    dir.__mul__(coef)
    return dir.__add__(new_dir)

  @staticmethod
  def goToPoint(robot: Robot, target: Point,
                 opponents: dict[int, Robot] = dict(),
                 teammates: dict[int, Robot] = dict()):
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

    min_dist, close_opponent = AgentNavigation.min_dist(robot, opponents)
    if(min_dist > MIN_DISTANCE_TO_TURN):

      if distance_to_target > ADJUST_ANGLE_MIN_DIST:
        v_angle = Geometry.abs_smallest_angle_diff(math.pi - ANGLE_EPSILON, d_theta)

        v_proportional = v_angle * (max_velocity / (math.pi - ANGLE_EPSILON))
        global_final_velocity = Geometry.from_polar(v_proportional, target_angle)
        target_velocity = Navigation.global_to_local_velocity(global_final_velocity.x, global_final_velocity.y, robot_angle)

        return target_velocity, -kp * d_theta
      else:
        return Point(0.0, 0.0), -kp * d_theta
      
    else:
      v_angle = Geometry.abs_smallest_angle_diff(math.pi - ANGLE_EPSILON, d_theta)

      v_proportional = v_angle * (max_velocity / (math.pi - ANGLE_EPSILON))
      new_dir = AgentNavigation.new_direction(robot, target, close_opponent)

      global_final_velocity = Geometry.from_polar(v_proportional, new_dir.angle())
      target_velocity = Navigation.global_to_local_velocity(global_final_velocity.x, global_final_velocity.y,robot_angle)

      return target_velocity, -kp * d_theta