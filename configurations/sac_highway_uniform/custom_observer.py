import numpy as np
from bark.models.dynamic import StateDefinition
from modules.runtime.commons.parameters import ParameterServer
import math
import operator
from src.commons.spaces import BoundedContinuous, Discrete

from src.observers.nearest_state_observer import ClosestAgentsObserver
# from configuration import SACHighwayConfiguration

class CustomObserver(ClosestAgentsObserver):
  def __init__(self, params=ParameterServer()):
    ClosestAgentsObserver.__init__(self,
                            params)
    self._perform_lane_change = False
    self._observation_len = \
      self._len_ego_state + \
        self._max_num_vehicles*self._len_relative_agent_state + 1

  def observe(self, world, agents_to_observe):
    extended_state = super(CustomObserver, self).observe(world, agents_to_observe)
    lane_change = world.agents[agents_to_observe[0]].goal_definition.lane_change
    extended_state[-1] = lane_change
    # extended_state[-1] = self._params["ML"]["Maneuver"]["slow_down"]
    return extended_state

  def reset(self, world, agents_to_observe):
    super(CustomObserver, self).reset(world, agents_to_observe)
    #rn = np.random.randint(0, 2)
    # rn = 0
    # sd = np.random.randint(0, 2)
    #self._params["ML"]["Maneuver"]["lane_change"] = rn
    # self._params["ML"]["Maneuver"]["slow_down"] = sd
    return world