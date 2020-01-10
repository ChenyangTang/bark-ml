import numpy as np
from bark.world.evaluation import \
  EvaluatorGoalReached, EvaluatorCollisionEgoAgent, \
  EvaluatorCollisionDrivingCorridor, EvaluatorStepCount
from modules.runtime.commons.parameters import ParameterServer
from bark.geometry import *
from bark.models.dynamic import StateDefinition

from src.evaluators.goal_reached import GoalReached


class CustomEvaluator(GoalReached):
  """Shows the capability of custom elements inside
     a configuration.
  """
  def __init__(self,
               params=ParameterServer(),
               eval_agent=None):
    GoalReached.__init__(self,
                         params,
                         eval_agent)
    self._last_goal_id = -1
    self._reached_goal_in_last_step = False
    # TODO: use from config file
    self._intermediate_goal_reward = 1.0

  def _add_evaluators(self):
    self._evaluators["goal_reached"] = EvaluatorGoalReached(self._eval_agent)
    self._evaluators["ego_collision"] = \
      EvaluatorCollisionEgoAgent(self._eval_agent)
    self._evaluators["step_count"] = EvaluatorStepCount()

  # def _distance_to_center_line(self, world, lane_change):
  #   """calculates the distance of the agent
  #      to its centerline
    
  #   Arguments:
  #       world {bark.world} -- bark world
    
  #   Returns:
  #       float -- distance to centerline
  #   """
  #   agent = world.agents[self._eval_agent]
  #   agent_state = agent.state
  #   centerline = agent.local_map.get_driving_corridor().center
    
  #   if lane_change == 1:
  #     agent_xy = Point2d(agent.state[1] - 4., agent.state[2])
      
  #   else:
  #     agent_xy = Point2d(agent.state[1], agent.state[2])
  #   return distance(centerline, agent_xy)

  def _distance_to_goal(self, world):
    d = 0.
    for i, agent in world.agents.items():
      state = agent.state
      goal_poly = agent.goal_definition.goal_shape
      d += distance(goal_poly, Point2d(state[1], state[2]))
    d /= i
    return d

  def _evaluate(self, world, eval_results):
    """Returns information about the current world state
    """
    # should read parameter that has been set in the observer
    
    agent_state = world.agents[self._eval_agent].state
    done = False
    agent = world.agents[self._eval_agent]

    lane_change = agent.goal_definition.lane_change
    current_goal_id = agent.goal_definition.GetCurrentId()

    # counts up for currently three added goals
    # print("current goal id", current_goal_id)
    # print("do lane change", lane_change)

    # this only counts up 
    intermediate_goal_reward = 0.

    # only give intermediate reward if the agent reaches the goal state continuously
    # if self._last_goal_id + 1 == current_goal_id:
    #   intermediate_goal_reward = 1.
    #   self._reached_goal_in_last_step = True
    #   if self._reached_goal_in_last_step is False:
    #     intermediate_goal_reward = 0.
    #     self._reached_goal_in_last_step = False 
    #   self._last_goal_id = current_goal_id
    # else:
    #   self._reached_goal_in_last_step = False

    # if self._last_goal_id + 1 == current_goal_id:
    #   if self._reached_goal_in_last_step is True:
    #     intermediate_goal_reward = 1.
    #   else:
    #     intermediate_goal_reward = 0.
    #   self._last_goal_id = current_goal_id
    #   self._reached_goal_in_last_step = True
    # else:
    #   self._reached_goal_in_last_step = False
    # print("last goal id", self._last_goal_id)

    success = eval_results["goal_reached"]
    distance = self._distance_to_goal(world)
    collision = eval_results["ego_collision"]
    step_count = eval_results["step_count"]
    # determine whether the simulation should terminate
    if success or collision or step_count > self._max_steps:
      done = True
    # calculate reward
    
    reward = collision * self._collision_penalty + \
      - 0.1*distance + \
      success * self._goal_reward
    # print("intermediate_reward = {}".format(str(intermediate_goal_reward)))
    # print("distance = {}".format(str(distance)))
    # print("reward = {}".format(str(reward)))

    return reward, done, eval_results
    
  def reset(self, world, agents_to_evaluate):
    world = super(CustomEvaluator, self).reset(world, agents_to_evaluate)
    self._eval_agent = agents_to_evaluate[0]
    self._last_goal_id = -1
    self._reached_goal_in_last_step = False
    return world
