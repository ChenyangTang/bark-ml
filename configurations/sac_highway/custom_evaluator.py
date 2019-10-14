import numpy as np
from bark.world.evaluation import \
  EvaluatorGoalReached, EvaluatorCollisionEgoAgent, \
  EvaluatorCollisionDrivingCorridor, EvaluatorStepCount
from modules.runtime.commons.parameters import ParameterServer
from bark.geometry import *

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

  def _add_evaluators(self):
    self._evaluators["goal_reached"] = EvaluatorGoalReached(self._eval_agent)
    self._evaluators["ego_collision"] = \
      EvaluatorCollisionEgoAgent(self._eval_agent)
    self._evaluators["corridor"] = EvaluatorCollisionDrivingCorridor()
    self._evaluators["step_count"] = EvaluatorStepCount()

  def _distance_to_center_line(self, world):
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

    lane_change = \
    self._params["ML"]["Maneuver"]["lane_change"]
    if lane_change == 1:
      agent_xy = Point2d(agent.state[1] - 4., agent.state[2])
    else:
      agent_xy = Point2d(agent.state[1], agent.state[2])
    return distance(centerline, agent_xy)

  def _evaluate(self, world, eval_results):
    """Returns information about the current world state
    """
    agent_state = world.agents[self._eval_agent].state
    # agent_velocity = np.sqrt((agent_state[4] - 10.)**2)
    done = False
    success = eval_results["goal_reached"]
    distance = self._distance_to_center_line(world)
    collision = eval_results["ego_collision"]
    step_count = eval_results["step_count"]
    # determine whether the simulation should terminate
    if success or collision or step_count > self._max_steps:
      done = True
    # calculate reward
    reward = collision * self._collision_penalty + \
      success * self._goal_reward - 0.1*distance #- 0.1*agent_velocity
    return reward, done, eval_results
    

