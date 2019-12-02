
from gym import spaces
import numpy as np
from bark.models.dynamic import StateDefinition
from bark.world import World, ObservedWorld
from src.commons.spaces import BoundedContinuous, Discrete
from modules.runtime.commons.parameters import ParameterServer
import math
import operator

from src.observers.observer import StateObserver
from source.graph_gen import generate_graph, generate_vehicles_and_graph


class GraphObserver(StateObserver):
  def __init__(self, params=ParameterServer()):
    StateObserver.__init__(self, params)
    self._state_definition = [int(StateDefinition.X_POSITION),
                              int(StateDefinition.Y_POSITION),
                              int(StateDefinition.THETA_POSITION),
                              int(StateDefinition.VEL_POSITION)]

  def observe(self, world, agents_to_observe):
    """see base class
    """
    vehicles = []
    observed_worlds = [world]
    if not isinstance(world, ObservedWorld):
      observed_worlds =  world.observe(agents_to_observe)
    ego_observed_world = observed_worlds[0]
    ego_state = ego_observed_world.ego_agent.state
    # TODO(@hart): normalize after graph generation
    vehicles.append(self._select_state_by_index(ego_state))
    for agent_id, agent in ego_observed_world.other_agents.items():
      # normalized_state = self._normalize(agent.state)
      reduced_state = self._select_state_by_index(agent.state)
      vehicles.append(reduced_state)
    print("vehicles: ", vehicles)
    normalization = {
      "x": {
        "pos": int(StateDefinition.X_POSITION),
        "range": self._world_x_range
      },
      "y": {
        "pos": int(StateDefinition.Y_POSITION),
        "range": self._world_y_range
      },
      "vel": {
        "pos": int(StateDefinition.VEL_POSITION),
        "range": [-25., 25.]
      },
      "theta": {
        "pos": int(StateDefinition.THETA_POSITION),
        "range": [-6.17, 6.17]
      },
      "dx": {
        "pos": int(StateDefinition.X_POSITION),
        "range": [-15, 15]
      },
      "dy": {
        "pos": int(StateDefinition.Y_POSITION),
        "range": [-200, 200]
      }
    }
    graph, num = generate_graph(
      np.array(vehicles),
      4,
      max_rows=80,
      normalization=normalization)
		# TODO(@hart): normalize edge values
    return graph

  @property
  def observation_space(self):
    # TODO(@hart): use from spaces.py
    return spaces.Box(low=0., high=1., shape=(80, 10))

  def _norm(self, agent_state, position, range):
    agent_state[int(position)] = \
      (agent_state[int(position)] - range[0])/(range[1]-range[0])
    return agent_state

  def _normalize(self, agent_state):
    agent_state = \
      self._norm(agent_state,
                 StateDefinition.X_POSITION,
                 self._world_x_range)
    agent_state = \
      self._norm(agent_state,
                 StateDefinition.Y_POSITION,
                 self._world_y_range)
    agent_state = \
      self._norm(agent_state,
                 StateDefinition.THETA_POSITION,
                 self._theta_range)
    agent_state = \
      self._norm(agent_state,
                 StateDefinition.VEL_POSITION,
                 self._velocity_range)
    return agent_state

  def _calculate_relative_agent_state(self, ego_agent_state, agent_state):
    return agent_state

  @property
  def _len_relative_agent_state(self):
    return len(self._state_definition)

  @property
  def _len_ego_state(self):
    return len(self._state_definition)