import sys
import logging
import time
import tensorflow as tf
import numpy as np
tf.compat.v1.enable_v2_behavior()

from tf_agents.drivers import dynamic_step_driver
from tf_agents.drivers import dynamic_episode_driver
from modules.runtime.commons.parameters import ParameterServer

from tf_agents.metrics import tf_metrics
from tf_agents.eval import metric_utils
from tf_agents.utils import common
from tf_agents.trajectories import time_step as ts

from src.runners.tfa_runner import TFARunner


logger = logging.getLogger()
# NOTE(@hart): this will print all statements
# logging.basicConfig(stream=sys.stdout, level=logging.DEBUG)

class SACRunner(TFARunner):
  """Runner that takes the runtime and agent
     and runs the training and evaluation as specified.
  """
  def __init__(self,
               runtime=None,
               agent=None, # should be a list
               params=ParameterServer(),
               unwrapped_runtime=None):
    TFARunner.__init__(self,
                       runtime=runtime,
                       agent=agent, # list of agents
                       params=params,
                       unwrapped_runtime=unwrapped_runtime)

  def train(self):
    """Wrapper that sets the summary writer.
       This enables a seamingless integration with TensorBoard.
    """
    # collect initial episodes
    self.collect_initial_episodes()
    # main training cycle
    if self._summary_writer is not None:
      with self._summary_writer.as_default():
        self._train()
    else:
      self._train()
  
  def evaluate(self):
    """Evaluates the agent
       Need to overwrite the class of the base function as the metric class somehow does
       not work.
    """
    global_iteration = self._agent[0]._agent._train_step_counter.numpy()
    logger.info("Evaluating the agent's performance in {} episodes."
      .format(str(self._params["ML"]["Runner"]["evaluation_steps"])))
    # Ticket (https://github.com/tensorflow/agents/issues/59) recommends
    # to do the rendering in the original environment
    rewards = []
    steps = []
    if self._unwrapped_runtime is not None:
      for _ in range(0, self._params["ML"]["Runner"]["evaluation_steps"]):
        state = self._unwrapped_runtime.reset()
        is_terminal = False
        while not is_terminal:
          action_step_0 = self._agent[0]._eval_policy.action(ts.transition(state, reward=0.0, discount=1.0))
          action_step_1 = self._agent[1]._eval_policy.action(ts.transition(state, reward=0.0, discount=1.0))
          state, reward, is_terminal, _ = self._unwrapped_runtime.step(action_step_0.action.numpy())
          rewards.append(reward)
          steps.append(1)
          state, reward, is_terminal, _ = self._unwrapped_runtime.step(action_step_1.action.numpy())
          rewards.append(reward)
          steps.append(1)
    mean_reward = np.sum(np.array(rewards))/len(rewards)
    mean_steps = np.sum(np.array(steps))/len(steps)
    tf.summary.scalar("mean_reward",
                      mean_reward,
                      step=global_iteration)
    tf.summary.scalar("mean_steps",
                      mean_steps,
                      step=global_iteration)
    logger.info(
      "The agent achieved on average {} reward and {} steps in \
      {} episodes." \
      .format(str(mean_reward),
              str(mean_steps),
              str(self._params["ML"]["Runner"]["evaluation_steps"])))

  def _train(self):
    """Trains the agent as specified in the parameter file
    """
    iterator = []
    for agent in self._agent:
      iterator.append(iter(agent._dataset))
    for _ in range(0, self._params["ML"]["Runner"]["number_of_collections"]):

      global_iteration = self._agent[0]._agent._train_step_counter.numpy()

      # we collect experiences based on the SacAgent
      for i in range(0, len(self._collection_driver)):
        # this no matter what always outputs an action 
        self._collection_driver[i].run()

      # self._collection_driver_1.run()
      for i, it in enumerate(iterator):
        experience, _ = next(it)
        self._agent[i]._agent.train(experience)

      if global_iteration % self._params["ML"]["Runner"]["evaluate_every_n_steps"] == 0:
        self.evaluate()
        for i, agent in enumerate(self._agent):
          agent.save()

  def visualize(self, num_episodes=1):
    # Ticket (https://github.com/tensorflow/agents/issues/59) recommends
    # to do the rendering in the original environment
    if self._unwrapped_runtime is not None:
      for _ in range(0, num_episodes):
        # print("NEW EPISODE")
        state = self._unwrapped_runtime.reset()
        is_terminal = False
        while not is_terminal:
          action_step_0 = self._agent[0]._eval_policy.action(ts.transition(state, reward=0.0, discount=1.0))
          action_step_1 = self._agent[1]._eval_policy.action(ts.transition(state, reward=0.0, discount=1.0))
          state, reward, is_terminal, _ = self._unwrapped_runtime.step(action_step_0.action.numpy())
          state, reward, is_terminal, _ = self._unwrapped_runtime.step(action_step_1.action.numpy())
          
          self._unwrapped_runtime.render()