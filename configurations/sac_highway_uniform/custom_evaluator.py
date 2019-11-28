import numpy as np
from bark.world.evaluation import \
  EvaluatorGoalReached, EvaluatorCollisionEgoAgent, \
  EvaluatorCollisionDrivingCorridor, EvaluatorStepCount
from modules.runtime.commons.parameters import ParameterServer
from bark.geometry import *

from src.evaluators.goal_reached import GoalReached
success = 0

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

  def _add_evaluators(self):
    self._evaluators["goal_reached"] = EvaluatorGoalReached(self._eval_agent)
    self._evaluators["ego_collision"] = \
      EvaluatorCollisionEgoAgent(self._eval_agent)
    self._evaluators["step_count"] = EvaluatorStepCount()

  def _distance_to_center_line(self, world, lane_change):
    """calculates the distance of the agent
       to its centerline
    
    Arguments:
        world {bark.world} -- bark world
    
    Returns:
        float -- distance to centerline
    """
    agent = world.agents[self._eval_agent]
    agent_state = agent.state
    centerline = agent.local_map.get_driving_corridor().center
    
    if lane_change == 1:
      agent_xy = Point2d(agent.state[1] + 4., agent.state[2])
      
    else:
      agent_xy = Point2d(agent.state[1], agent.state[2])
    return distance(centerline, agent_xy)

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
    print("current goal id", current_goal_id)
    print("do lane change", lane_change)

    # calculate number of goals that have been reached
    number = current_goal_id + 1
    success = eval_results["goal_reached"]
    distance = self._distance_to_center_line(world, lane_change)
    collision = eval_results["ego_collision"]
    step_count = eval_results["step_count"]
    # determine whether the simulation should terminate
    if success or collision or step_count > self._max_steps:
      done = True
    # calculate reward
    #print("success1 = {}".format(str(success)))
    #print("goal = {}".format(str(self._goal_reward)))
    #print("distance = {}".format(str(distance)))
    #print("collision = {}".format(str(collision)))
    reward = collision * self._collision_penalty + \
      number * self._goal_reward - 0.1*distance
    print("reward = {}".format(str(reward)))

    return reward, done, eval_results
    

