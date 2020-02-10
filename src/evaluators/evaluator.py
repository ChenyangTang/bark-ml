from abc import ABC, abstractmethod

from modules.runtime.commons.parameters import ParameterServer

class StateEvaluator(ABC):
  """Evaluates the state of the environment
     e.g., if a collision has happend
  """
  def __init__(self,
               params=ParameterServer()):
    self._params = params
    self._evaluators = {}

  def evaluate(self, world, action, agent_id):
    """Evaluates the passed world
    
    Arguments:
        world {bark.world} -- World containing all information
    
    Returns:
        (reward, status, evaluation results) -- Rl-tuple
    """
    eval_results = None
    reward = 0.
    done = False
    if self._eval_agent[0] in world.agents and self._eval_agent[1] in world.agents:
      eval_results = world.evaluate()
      reward, done, eval_results = self._evaluate(world, eval_results, action, agent_id)
    return reward, done, eval_results

  def reset(self, world, agents_to_evaluate):
    self._eval_agent = agents_to_evaluate
    world.clear_evaluators()
    self._add_evaluators()
    for key, evaluator in self._evaluators.items():
      world.add_evaluator(key, evaluator)
    return world