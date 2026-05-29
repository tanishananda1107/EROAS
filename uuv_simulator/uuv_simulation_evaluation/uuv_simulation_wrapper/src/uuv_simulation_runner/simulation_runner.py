import logging
import os
import sys
import yaml
import time
import random
import shutil
import psutil
import datetime
import signal
import socket
import subprocess

from threading import Timer
from time import gmtime, strftime, sleep


PORT_LOCK_FILE = 'uuv_port_lock'


class SimulationRunner:

    def __init__(
        self,
        params,
        task_filename,
        results_folder='./results',
        record_all_results=False,
        add_folder_timestamp=True,
        log_filename=None,
        log_dir='logs'
    ):

        self._task_name = os.path.splitext(os.path.basename(task_filename))[0]

        self.record_all_results = record_all_results
        self._log_dir = log_dir

        os.makedirs(log_dir, exist_ok=True)

        self._logger = logging.getLogger(
            f'simulation_runner_{self._task_name}'
        )

        if not self._logger.handlers:

            handler = logging.StreamHandler(sys.stdout)

            fmt = logging.Formatter(
                '%(asctime)s | %(levelname)s | %(message)s'
            )

            handler.setFormatter(fmt)

            self._logger.addHandler(handler)

            self._logger.setLevel(logging.INFO)

        assert isinstance(params, dict)
        self._params = params

        self._task_filename = task_filename

        assert os.path.isfile(task_filename)

        with open(task_filename, 'r') as f:
            self._task_text = f.read()

        self._results_folder = os.path.abspath(results_folder)

        os.makedirs(self._results_folder, exist_ok=True)

        self._sim_results_dir = None
        self._recording_filename = None

        self._process = None
        self._process_children = []

        self._timeout = 100000
        self._simulation_timeout = None

        self._process_timeout_triggered = False
        self.processes_interrupted = False

        self._add_folder_timestamp = add_folder_timestamp

        signal.signal(signal.SIGTERM, self.signal_handler)
        signal.signal(signal.SIGINT, self.signal_handler)

        self._logger.info("ROS2 SimulationRunner initialized")

    # -------------------------
    # SIGNAL HANDLING
    # -------------------------
    def signal_handler(self, sig, frame):
        self._logger.warning(f"Signal received: {sig}")
        self.processes_interrupted = True
        self._kill_process()

    # -------------------------
    # PROCESS CONTROL
    # -------------------------
    def _kill_process(self):

        if self._process is None:
            return

        self._logger.warning("Killing simulation process tree...")

        try:
            parent = psutil.Process(self._process.pid)

            children = parent.children(recursive=True)

            for p in children:
                p.terminate()

            parent.terminate()

            gone, alive = psutil.wait_procs(children, timeout=5)

            self._process_timeout_triggered = True

        except Exception as e:
            self._logger.error(f"Kill error: {e}")

    # -------------------------
    # ENV (ROS2 + GZ SIM 8)
    # -------------------------
    def _set_env(self):

        # ROS2 does NOT use ROS_MASTER_URI
        # Only Gazebo Harmonic uses gz sim env if needed

        if self._sim_results_dir:
            os.environ["ROS_LOG_DIR"] = os.path.join(
                self._sim_results_dir,
                "ros_logs"
            )

    # -------------------------
    # MAIN RUN FUNCTION
    # -------------------------
    def run(self, params=None, timeout=None):

        if params:
            self._params.update(params)

        self._sim_results_dir = os.path.join(
            self._results_folder,
            self._task_name + "_" +
            strftime("%Y-%m-%d_%H-%M-%S", gmtime()) +
            "_" + str(random.randint(0, 1000))
        ) if self._add_folder_timestamp else self._results_folder

        os.makedirs(self._sim_results_dir, exist_ok=True)

        self._set_env()

        task_file = os.path.join(self._sim_results_dir, "task.yaml")

        with open(task_file, "w") as f:
            f.write(self._task_text)

        task = yaml.safe_load(open(task_file))

        self._logger.info(f"Running task: {task.get('id', 'unknown')}")

        # -------------------------
        # GAZEBO HARMONIC COMMAND
        # -------------------------
        # Example assumes gz sim 8 world launch
        cmd = task["execute"]["cmd"]

        # convert ROS1-style params -> ROS2 CLI style (still simple passthrough)
        for k, v in task["execute"].get("params", {}).items():
            if isinstance(v, bool):
                v = int(v)

            cmd += f" {k}:={v}"

        # add runtime params
        for k, v in self._params.items():
            cmd += f" {k}:={v}"

        self._recording_filename = os.path.join(
            self._sim_results_dir,
            "recording.log"   # ROS2 replaces bag in this simple version
        )

        cmd += f" > {self._recording_filename} 2>&1"

        self._logger.info(f"Executing: {cmd}")

        # -------------------------
        # START PROCESS (ROS2 + GZ SIM 8)
        # -------------------------
        self._process = psutil.Popen(cmd, shell=True)

        proc = psutil.Process(self._process.pid)

        self._process_children = proc.children(recursive=True)

        # timeout control
        timer = Timer(timeout or self._timeout, self._kill_process)
        timer.start()

        try:
            exit_code = self._process.wait(timeout=timeout or self._timeout)

            success = (exit_code == 0)

        except Exception:
            self._kill_process()
            success = False

        self._logger.info(
            f"Simulation finished: {self._sim_results_dir}"
        )

        self._process = None

        return success
