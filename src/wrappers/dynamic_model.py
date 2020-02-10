
import numpy as np

from src.commons.spaces import Discrete, BoundedContinuous
from bark.models.behavior import DynamicBehaviorModel
from bark.models.dynamic import SingleTrackModel, TripleIntegratorModel
from modules.runtime.commons.parameters import ParameterServer
from src.wrappers.action_wrapper import ActionWrapper

class DynamicModel(ActionWrapper):
  """This module wraps the SingleTrack model
     and requires the steering angle and acceleration
     as system inputs. 
  """
  def __init__(self,
               model_name="SingleTrackModel",
               params=ParameterServer()):
    ActionWrapper.__init__(self, params)
    self._control_inputs = \
      self._params["ML"]["DynamicModel"]["action_dimension",
      "Dimension of action",
      2]
    self._dynamic_model = eval("{}(self._params)".format(model_name))
    self._behavior_models = []
    self._input_count = 0

  def reset(self, world, agents_to_act):
    """see base class
    """
    super(DynamicModel, self).reset(world=world,
                                    agents_to_act=agents_to_act)
    self._input_count = 0
    self._behavior_models = []
    for agent_id in agents_to_act:
      self._behavior_models.append(DynamicBehaviorModel(self._dynamic_model,
                                                        self._params))
      if agent_id in world.agents:
        world.agents[agent_id].behavior_model = self._behavior_models[-1]
        world.agents[agent_id].set_move_automatically(False)
      else:
        raise ValueError("AgentID does not exist in world.")
    return world

  def action_to_behavior(self, world, action):
    """see base class
    """
    if len(self._behavior_models) >= 1:
      # set_last_action
      self._behavior_models[self._input_count].set_last_action(action)
      self._input_count += 1
    return world

  @property
  def action_space(self):
    """see base class
    """
    return BoundedContinuous(
      self._control_inputs,
      low=self._params["ML"]["DynamicModel"]["actions_lower_bound",
        "Lower-bound for actions.",
        [0.5, -0.01, -0.1]],
      high=self._params["ML"]["DynamicModel"]["actions_upper_bound",
        "Upper-bound for actions.",
        [0.5, 0.01, 0.1]])