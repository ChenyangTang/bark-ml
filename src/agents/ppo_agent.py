import tensorflow as tf

# tfa
from tf_agents.networks import actor_distribution_network
from tf_agents.networks import value_network
from tf_agents.policies import greedy_policy

from tf_agents.agents.ppo import ppo_agent
from tf_agents.replay_buffers import tf_uniform_replay_buffer
from tf_agents.utils.common import Checkpointer
from tf_agents.trajectories import time_step as ts

from src.agents.tfa_agent import TFAAgent

class PPOAgent(TFAAgent):
  """PPO-Agent
     This agent is based on the tf-agents library.
  """
  def __init__(self,
               environment=None,
               replay_buffer=None,
               checkpointer=None,
               dataset=None,
               params=None):
    TFAAgent.__init__(self,
                      environment=environment,
                      params=params)
    self._replay_buffer = self.get_replay_buffer()
    # self._dataset = self.get_dataset()
    self._collect_policy = self.get_collect_policy()
    self._eval_policy = self.get_eval_policy()

  def get_agent(self, env, params):
    """Returns a TensorFlow PPO-Agent
    
    Arguments:
        env {TFAPyEnvironment} -- Tensorflow-Agents PyEnvironment
        params {ParameterServer} -- ParameterServer from BARK
    
    Returns:
        agent -- tf-agent
    """

    # actor network
    actor_net = actor_distribution_network.ActorDistributionNetwork(
        env.observation_spec(),
        env.action_spec(),
        fc_layer_params=tuple(
          self._params["ML"]["Agent"]["actor_fc_layer_params"]))

    # critic network
    value_net = value_network.ValueNetwork(
      env.observation_spec(),
      fc_layer_params=tuple(
        self._params["ML"]["Agent"]["critic_fc_layer_params"]))
    
    # agent
    tf_agent = ppo_agent.PPOAgent(
      env.time_step_spec(),
      env.action_spec(),
      actor_net=actor_net,
      value_net=value_net,
      optimizer=tf.compat.v1.train.AdamOptimizer(
          learning_rate=self._params["ML"]["Agent"]["learning_rate"]),
      train_step_counter=self._ckpt.step,
      num_epochs=self._params["ML"]["Agent"]["num_epochs"],
      name=self._params["ML"]["Agent"]["agent_name"],
      debug_summaries=self._params["ML"]["Agent"]["debug_summaries"])
    tf_agent.initialize()
    return tf_agent

  def get_replay_buffer(self):
    """Replay buffer
    
    Returns:
        ReplayBuffer -- tf-agents replay buffer
    """
    return tf_uniform_replay_buffer.TFUniformReplayBuffer(
      data_spec=self._agent.collect_data_spec,
      batch_size=self._params["ML"]["Agent"]["num_parallel_environments"],
      max_length=self._params["ML"]["Agent"]["replay_buffer_capacity"])

  # def get_dataset(self):
  #   """Dataset generated of the replay buffer
    
  #   Returns:
  #       dataset -- subset of experiences
  #   """
  #   dataset = self._replay_buffer.as_dataset(
  #     num_parallel_calls=self._params["ML"]["Agent"]["parallel_buffer_calls"],
  #     sample_batch_size=self._params["ML"]["Agent"]["batch_size"],
  #     num_steps=self._params["ML"]["Agent"]["buffer_num_steps"]) \
  #       .prefetch(self._params["ML"]["Agent"]["buffer_prefetch"])
  #   return dataset

  def get_collect_policy(self):
    """Returns the collection policy of the agent
    
    Returns:
        CollectPolicy -- Samples from the agent's distribution
    """
    return self._agent.collect_policy

  def get_eval_policy(self):
    """Returns the greedy policy of the agent
    
    Returns:
        GreedyPolicy -- Always returns best suitable action
    """
    return greedy_policy.GreedyPolicy(self._agent.policy)

  def reset(self):
    pass

  @property
  def collect_policy(self):
    return self._collect_policy

  @property
  def eval_policy(self):
    return self._eval_policy

  def act(self, state):
    """ see base class
    """
    action_step = self.eval_policy.action(
      ts.transition(state, reward=0.0, discount=1.0))
    return action_step.action.numpy()
