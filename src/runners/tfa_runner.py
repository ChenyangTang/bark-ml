import sys
import logging
import time
import tensorflow as tf
tf.compat.v1.enable_v2_behavior()

from tf_agents.drivers import dynamic_step_driver
from tf_agents.drivers import dynamic_episode_driver
from modules.runtime.commons.parameters import ParameterServer

from tf_agents.metrics import tf_metrics
from tf_agents.eval import metric_utils
from tf_agents.utils import common
from tf_agents.trajectories import time_step as ts

from src.runners.base_runner import BaseRunner


logger = logging.getLogger()
# NOTE(@hart): this will print all statements
# logging.basicConfig(stream=sys.stdout, level=logging.DEBUG)

class TFARunner(BaseRunner):
  """Runner that takes the runtime and agent
     and runs the training and evaluation as specified.
  """
  def __init__(self,
               runtime=None,
               agent=None,
               params=ParameterServer(),
               unwrapped_runtime=None):
    BaseRunner.__init__(self,
                        runtime=runtime,
                        agent=agent,
                        params=params)
    self._eval_metrics = [
      tf_metrics.AverageReturnMetric(
        buffer_size=self._params["ML"]["Runner"]["evaluation_steps"]),
      tf_metrics.AverageEpisodeLengthMetric(
        buffer_size=self._params["ML"]["Runner"]["evaluation_steps"])
    ]
    self._summary_writer = None
    self._unwrapped_runtime = unwrapped_runtime
    if self._params["ML"]["Runner"]["summary_path"] is not None:
      self._summary_writer = tf.summary.create_file_writer(
        self._params["ML"]["Runner"]["summary_path"])
    self.get_initial_collection_driver()
    self.get_collection_driver()

  def get_initial_collection_driver(self):
    """Sets the initial collection driver for tf-agents.
    """
    self._initial_collection_driver = \
      dynamic_episode_driver.DynamicEpisodeDriver(
        env=self._runtime,
        policy=self._agent._agent.collect_policy,
        observers=[self._agent._replay_buffer.add_batch],
        num_episodes=self._params["ML"]["Runner"]["initial_collection_steps"])
    self._initial_collection_driver.run = common.function(
      self._initial_collection_driver.run)

  def get_collection_driver(self):
    """Sets the collection driver for tf-agents.
    """
    self._collection_driver = dynamic_episode_driver.DynamicEpisodeDriver(
      env=self._runtime,
      policy=self._agent._agent.collect_policy,
      observers=[self._agent._replay_buffer.add_batch],
      num_episodes=self._params["ML"]["Runner"]["collection_episodes_per_cycle"])
    self._collection_driver.run = common.function(self._collection_driver.run)

  def collect_initial_episodes(self):
    """Function that collects the initial episodes
    """
    self._initial_collection_driver.run()

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

  def _train(self):
    """Trains the agent as specified in the parameter file
    """
    iterator = iter(self._agent._dataset)
    for _ in range(0, self._params["ML"]["Runner"]["number_of_collections"]):
      global_iteration = self._agent._agent._train_step_counter.numpy()
      self._collection_driver.run()
      experience, _ = next(iterator)
      self._agent._agent.train(experience)
      if global_iteration % self._params["ML"]["Runner"]["evaluate_every_n_steps"] == 0:
        self.evaluate()
        self._agent.save()

  def evaluate(self):
    """Evaluates the agent
    """
    global_iteration = self._agent._agent._train_step_counter.numpy()
    logger.info("Evaluating the agent's performance in {} episodes."
      .format(str(self._params["ML"]["Runner"]["evaluation_steps"])))
    metric_utils.eager_compute(
      self._eval_metrics,
      self._runtime,
      self._agent._agent.policy,
      num_episodes=self._params["ML"]["Runner"]["evaluation_steps"])
    metric_utils.log_metrics(self._eval_metrics)
    tf.summary.scalar("mean_reward",
                      self._eval_metrics[0].result().numpy(),
                      step=global_iteration)
    tf.summary.scalar("mean_steps",
                      self._eval_metrics[1].result().numpy(),
                      step=global_iteration)
    logger.info(
      "The agent achieved on average {} reward and {} steps in \
      {} episodes." \
      .format(str(self._eval_metrics[0].result().numpy()),
              str(self._eval_metrics[1].result().numpy()),
              str(self._params["ML"]["Runner"]["evaluation_steps"])))

  def visualize(self, num_episodes=1):
    # Ticket (https://github.com/tensorflow/agents/issues/59) recommends
    # to do the rendering in the original environment
    if self._unwrapped_runtime is not None:
      for _ in range(0, num_episodes):
        state = self._unwrapped_runtime.reset()
        is_terminal = False
        while not is_terminal:
          action_step = self._agent._eval_policy.action(ts.transition(state, reward=0.0, discount=1.0))
          print("State: {}".format(state))
          # TODO(@hart); make generic for multi agent planning
          state, _, is_terminal, _ = self._unwrapped_runtime.step(action_step.action.numpy())
          self._unwrapped_runtime.render()
          # TODO(@hart): could add flag for real-time-feel visualization
          time.sleep(0.001)
