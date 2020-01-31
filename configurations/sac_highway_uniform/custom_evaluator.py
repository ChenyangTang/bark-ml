import numpy as np
from bark.world.evaluation import \
  EvaluatorGoalReached, EvaluatorCollisionAgents, EvaluatorCollisionEgoAgent, \
  EvaluatorCollisionDrivingCorridor, EvaluatorStepCount, EvaluatorDrivableArea
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
    self._evaluators["goal_reached"] = EvaluatorGoalReached()
    self._evaluators["drivable_area"] = EvaluatorDrivableArea()
    # self._evaluators["ego_collision"] = \
    #   EvaluatorCollisionEgoAgent(self._eval_agent)
    self._evaluators["collision_agent_0"] = \
      EvaluatorCollisionAgents()
    self._evaluators["step_count"] = EvaluatorStepCount()

  def _distance_to_goal(self, world, agent_id):
    d = 0.
    agent = world.agents[agent_id]
    state = agent.state
    goal_poly = agent.goal_definition.goal_shape
    d += distance(goal_poly, Point2d(state[1], state[2]))
    return d
  
  def deviation_velocity(self, world, agent_id):
    desired_v = 11.
    delta_v = 0.
    agent = world.agents[agent_id]
    vel = agent.state[int(StateDefinition.VEL_POSITION)]
    delta_v += (desired_v-vel)**2
    return delta_v

  def _evaluate(self, world, eval_results, action, agent_id):
    """Returns information about the current world state
    """
    # TODO(@Chenyang):check if intermediate reward is still needed
    
    # agent_state = world.agents[self._eval_agent].state
    # agent = world.agents[self._eval_agent]

    # lane_change = agent.goal_definition.lane_change
    # current_goal_id = agent.goal_definition.GetCurrentId()

    # counts up for currently three added goals
    # print("current goal id", current_goal_id)
    # print("do lane change", lane_change)

    # this only counts up 
    # intermediate_goal_reward = 0.

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

    done = False

    success = eval_results["goal_reached"]
    distance = self._distance_to_goal(world, agent_id)
    collision = eval_results["collision_agent_0"]
    step_count = eval_results["step_count"]
    drivable_area = eval_results["drivable_area"]

    # determine whether the simulation should terminate
    if success or collision or step_count > self._max_steps: # or drivable_area:
      done = True
      
    # calculate reward
    reward = collision * self._collision_penalty + \
      - 0.1*distance + \
      success * self._goal_reward - self.deviation_velocity(world, agent_id)
      #+ drivable_area * self._collision_penalty 

    return reward, done, eval_results
    
  def reset(self, world, agents_to_evaluate):
    world = super(CustomEvaluator, self).reset(world, agents_to_evaluate)
    self._eval_agent = agents_to_evaluate
    self._last_goal_id = -1
    self._reached_goal_in_last_step = False
    return world
