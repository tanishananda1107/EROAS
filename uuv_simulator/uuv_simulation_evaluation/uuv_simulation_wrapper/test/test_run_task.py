import unittest
import os
import shutil
import yaml

from uuv_simulation_runner import SimulationRunner


PARAMS = dict(
    Kp=[11993.888, 11993.888, 11993.888, 19460.069, 19460.069, 19460.069],
    Kd=[9077.459, 9077.459, 9077.459, 18880.925, 18880.925, 18880.925],
    Ki=[321.417, 321.417, 321.417, 2096.951, 2096.951, 2096.951],
)

OCEAN_WORLD_TASK = os.path.join(
    os.path.dirname(__file__),
    'example_start_ocean_world.yaml'
)

OUTPUT_DIR = '/tmp'
RESULTS_DIR = os.path.join(OUTPUT_DIR, 'results')


class TestRunTask(unittest.TestCase):

    def tearDown(self):

        if os.path.isdir(RESULTS_DIR):
            shutil.rmtree(RESULTS_DIR)

    def test_create_task(self):

        runner = SimulationRunner(
            PARAMS,
            OCEAN_WORLD_TASK,
            RESULTS_DIR,
            True
        )

        runner.run(PARAMS)

        self.assertTrue(os.path.isdir(RESULTS_DIR))

        self.assertTrue(
            os.path.isdir(runner.current_sim_results_dir)
        )

        del runner

    def test_params(self):

        runner = SimulationRunner(
            PARAMS,
            OCEAN_WORLD_TASK,
            RESULTS_DIR,
            True
        )

        runner.run(PARAMS)

        param_file = os.path.join(
            runner.current_sim_results_dir,
            'params_0.yml'
        )

        self.assertTrue(os.path.exists(param_file))

        with open(param_file, 'r') as f:
            params = yaml.safe_load(f)

        for k in PARAMS:
            self.assertIn(k, params)
            self.assertEqual(params[k], PARAMS[k])

        del runner

    def test_timeout(self):

        runner = SimulationRunner(
            PARAMS,
            OCEAN_WORLD_TASK,
            RESULTS_DIR,
            True
        )

        success = runner.run(PARAMS, timeout=2)

        self.assertFalse(success)

        del runner

    def test_batch(self):

        for _ in range(3):

            runner = SimulationRunner(
                {},
                OCEAN_WORLD_TASK,
                RESULTS_DIR,
                True
            )

            success = runner.run({}, timeout=30)

            self.assertTrue(success)

            del runner
