# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT


import unittest
import tensorflow as tf
from tf_agents.environments import tf_py_environment

from modules.runtime.scenario.scenario_generation.uniform_vehicle_distribution import UniformVehicleDistribution
from modules.runtime.commons.parameters import ParameterServer
from modules.runtime.viewer.matplotlib_viewer import MPViewer

from src.rl_runtime import RuntimeRL
from src.observers.nn_state_observer import StateConcatenation
from src.wrappers.action_wrapper import DynamicModel
from src.wrappers.tfa_wrapper import TFAWrapper
from src.evaluators.state_evaluator import GoalReached
from src.agents.sac_agent import SACAgent
from src.drivers.tfa_runner import TFARunner

tf.compat.v1.enable_v2_behavior()

class RunnerTests(unittest.TestCase):
  @staticmethod
  def test_runner():
    params = ParameterServer(filename="data/highway_merging.json")
    scenario_generation = UniformVehicleDistribution(num_scenarios=2, random_seed=0, params=params)
    state_observer = StateConcatenation(params=params)
    action_wrapper = DynamicModel(params=params)
    evaluator = GoalReached(params=params)
    viewer = MPViewer(params=params, x_range=[-30,30], y_range=[-20,40], follow_agent_id=True) # use_world_bounds=True
    runtimerl = RuntimeRL(action_wrapper=action_wrapper, nn_observer=state_observer,
                          evaluator=evaluator, step_time=0.05, viewer=viewer,
                          scenario_generator=scenario_generation)
    tfa_env = tf_py_environment.TFPyEnvironment(TFAWrapper(runtimerl))
    # this is a fully working sac agent (agent, replay buffer, dataset, ...)
    sac_agent = SACAgent(tfa_env)
    tfa_runner = TFARunner(tfa_env, sac_agent)
    tfa_runner.collect_initial_episodes()
    tfa_runner.train()

if __name__ == '__main__':
    unittest.main()