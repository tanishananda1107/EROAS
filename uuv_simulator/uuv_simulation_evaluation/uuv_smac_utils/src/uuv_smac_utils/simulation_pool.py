# Copyright (c) 2016 The UUV Simulator Authors.
# All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# ROS 2 / Gazebo Harmonic (gz-sim 8) port
# Changes from ROS 1:
#
#   1. uuv_bag_evaluation  → uuv_bag_evaluation2
#      The Evaluation class now wraps rosbag2_py instead of rosbag.
#
#   2. uuv_simulation_runner → uuv_simulation_runner2
#      SimulationRunner2 must use `ros2 launch` instead of `roslaunch`,
#      and its recording_filename attribute now points to a rosbag2
#      *directory* (not a single .bag file).
#
#   3. rosbag2 recording detection: bag files are no longer single .bag files.
#      rosbag2 writes a directory containing metadata.yaml + data files
#      (*.db3 for SQLite3 storage, *.mcap for MCAP storage).
#      Detection logic updated:
#        ROS 1:  look for items ending with '.bag' or '.bag.active'
#        ROS 2:  look for 'metadata.yaml' inside the recording directory,
#                OR for any '.db3' / '.mcap' data file.
#      The `.bag.active` → active-write sentinel is replaced by checking
#      for the absence of metadata.yaml (rosbag2 writes it only on close).

import os
import yaml
import logging
import sys
import datetime
import signal
from copy import deepcopy
from time import sleep
import random
import shutil
from .utils import *
# ROS 2: updated package names
from uuv_simulation_runner2 import SimulationRunner
from uuv_bag_evaluation2 import Evaluation
from multiprocessing import Pool, Lock, Value
from .opt_configuration import OptConfiguration

N_SIMULATION_RUNS = Value('i', 0)
N_SUCCESS = Value('i', 0)
N_CRASHES = Value('i', 0)
TERMINATE_ALL_PROCESSES = Value('i', 0)

PROCESS_LOCK = Lock()

THREAD_POOL = None


def signal_handler(signal, frame):
    SIMULATION_LOGGER.warning('SIGNAL RECEIVED=%d', int(signal))
    if THREAD_POOL is not None:
        SIMULATION_LOGGER.warning('Finishing all processes in the simulation pool')
        with TERMINATE_ALL_PROCESSES.get_lock():
            TERMINATE_ALL_PROCESSES.value = 1
        THREAD_POOL.terminate()
        THREAD_POOL.join()
        SIMULATION_LOGGER.warning('SIGTERM sent to all processes')


signal.signal(signal.SIGTERM, signal_handler)
signal.signal(signal.SIGINT, signal_handler)


def add_to_crash_log(data):
    assert isinstance(data, dict)

    with N_SIMULATION_RUNS.get_lock():
        N_SIMULATION_RUNS.value += 1
    with N_CRASHES.get_lock():
        N_CRASHES.value += 1

    SIMULATION_LOGGER.error('CRASHED - Simulation failed, info=')
    for tag in data:
        SIMULATION_LOGGER.error('\t%s=%s' % (tag, data[tag]))

    SIMULATION_LOGGER.info('# simulation runs=%d' % N_SIMULATION_RUNS.value)
    SIMULATION_LOGGER.info('\tSUCCESS=%d' % N_SUCCESS.value)
    SIMULATION_LOGGER.info('\tCRASHED=%d' % N_CRASHES.value)


def add_to_run_log(data):
    assert isinstance(data, dict)

    with N_SIMULATION_RUNS.get_lock():
        N_SIMULATION_RUNS.value += 1
    with N_SUCCESS.get_lock():
        N_SUCCESS.value += 1

    SIMULATION_LOGGER.info('SUCCESS - Simulation finished successfully, info=')
    for tag in data:
        SIMULATION_LOGGER.info('\t%s=%s' % (tag, data[tag]))

    SIMULATION_LOGGER.info('# simulation runs=%d' % N_SIMULATION_RUNS.value)
    SIMULATION_LOGGER.info('\tSUCCESS=%d' % N_SUCCESS.value)
    SIMULATION_LOGGER.info('\tCRASHED=%d' % N_CRASHES.value)


def _recording_is_complete(recording_dir):
    """Return True when a rosbag2 recording directory is fully written.

    rosbag2 writes metadata.yaml only after the recorder is cleanly stopped.
    Polling for metadata.yaml is therefore the rosbag2 equivalent of waiting
    for the '.bag.active' sentinel to disappear in ROS 1.

    Supported storage plugins: sqlite3 (*.db3) and MCAP (*.mcap).
    """
    if not os.path.isdir(recording_dir):
        return False
    metadata_path = os.path.join(recording_dir, 'metadata.yaml')
    return os.path.isfile(metadata_path)


def _recording_exists(recording_dir):
    """Return True as soon as a rosbag2 directory contains at least one data file.

    This replaces the ROS 1 check for '.bag' or '.bag.active' files.
    A data file appearing before metadata.yaml means recording is in progress.
    """
    if not os.path.isdir(recording_dir):
        return False
    for item in os.listdir(recording_dir):
        if item.endswith('.db3') or item.endswith('.mcap') or item == 'metadata.yaml':
            return True
    return False


def run_simulation(task):
    if TERMINATE_ALL_PROCESSES.value == 1:
        SIMULATION_LOGGER.warning('Process pool has been terminated, '
                                  'finishing simulation process')
        return dict()

    random.seed()
    sleep(random.random())
    opt_config = OptConfiguration.get_instance()

    SIMULATION_LOGGER.info('Starting simulation for task <%s>...' % task)
    SIMULATION_LOGGER.info('\tParameters=')
    for tag in sorted(opt_config.params.keys()):
        SIMULATION_LOGGER.info('\t - %s=%s' % (tag, str(opt_config.params[tag])))
    SIMULATION_LOGGER.info('\tPartial results root directory=' + opt_config.results_dir)
    SIMULATION_LOGGER.info('\tRecord all partial results? ' + str(opt_config.record_all))

    runner = None
    sim_eval = None
    try:
        runner = SimulationRunner(
            opt_config.params, task, opt_config.results_dir, opt_config.record_all)
        runner.run(opt_config.params)
        sleep(random.random() * 5)

        # ROS 2: recording_dirname is the rosbag2 output directory itself.
        # SimulationRunner2.recording_filename holds the path to that directory.
        recording_dirname = runner.recording_filename

        # Check whether rosbag2 has started writing data files.
        has_recording = _recording_exists(recording_dirname)

        if not has_recording:
            SIMULATION_LOGGER.error('No recording generated for task <%s>' % task)
        else:
            # Wait for rosbag2 to finish writing (metadata.yaml appears on close).
            has_recording = False
            for _ in range(30):
                if _recording_is_complete(recording_dirname):
                    has_recording = True
                    break
                sleep(0.1)

        if not has_recording:
            raise Exception(
                'No complete recording generated for task <%s>, dir=%s'
                % (task, recording_dirname))

    except Exception as e:
        SIMULATION_LOGGER.error(
            'Error occurred in this iteration, setting simulation status to '
            'CRASHED for task <%s>, message=%s' % (task, str(e)))
        status = SIM_CRASHED
        partial_cost = 1e7

        add_to_crash_log(dict(
            status=status,
            timestamp=str(datetime.datetime.now().isoformat()),
            results_dir=runner.current_sim_results_dir if runner else 'N/A',
            message=str(e),
            task=str(task)))

        output = dict(
            task=str(task),
            status=status,
            cost=partial_cost,
            sim_time=None,
            message=str(e),
            results_dir=runner.current_sim_results_dir if runner else 'N/A')

        if runner is not None:
            if not runner.record_all_results:
                runner.remove_recording_dir()
            del runner
        return output
    else:
        SIMULATION_LOGGER.info('Simulation finished, task=%s' % task)

    try:
        PROCESS_LOCK.acquire()

        time_offset = 0.0
        if opt_config.evaluation_time_offset is not None:
            time_offset = max(0.0, opt_config.evaluation_time_offset)

        SIMULATION_LOGGER.info('Start evaluation of the results')
        SIMULATION_LOGGER.info('\tTime offset for KPI evaluation[s]=' + str(time_offset))
        SIMULATION_LOGGER.info('\tResults files directory=' + runner.current_sim_results_dir)
        # ROS 2: recording_filename is a directory path, not a .bag file path.
        SIMULATION_LOGGER.info('\tROS 2 bag directory=' + runner.recording_filename)
        sim_eval = Evaluation(
            runner.recording_filename,
            runner.current_sim_results_dir,
            time_offset=time_offset)

        SIMULATION_LOGGER.info('Evaluation finished')

        sim_eval.compute_kpis()

        if opt_config.store_kpis_only:
            sim_eval.save_kpis()
            SIMULATION_LOGGER.info('Store KPIs only')
        else:
            sim_eval.save_evaluation()
            SIMULATION_LOGGER.info('Store KPIs and graphs')

        SIMULATION_LOGGER.info('Calculating cost function')

        kpis = sim_eval.get_kpis()

        for tag in kpis:
            if kpis[tag] < 0:
                SIMULATION_LOGGER.info(
                    'KPI <%s> returned an invalid value=%.3f' % (tag, kpis[tag]))
                raise Exception(
                    'KPI <%s> returned an invalid value=%.3f' % (tag, kpis[tag]))

        partial_cost = opt_config.compute_cost_fcn(sim_eval.get_kpis())

        if partial_cost < 0:
            raise Exception('Cost function returned value lower than zero')

        sim_time = float(runner.timeout - time_offset)

        opt_config.cost_fcn.save(runner.current_sim_results_dir)

        status = SIM_SUCCESS
        output = dict(
            timestamp=str(datetime.datetime.now().isoformat()),
            status=status,
            cost=float(partial_cost),
            sim_time=sim_time,
            results_dir=runner.current_sim_results_dir,
            recording_filename=runner.recording_filename,
            cost_function_data=opt_config.cost_fcn.get_data(),
            task=task)

        add_to_run_log(output)

        SIMULATION_LOGGER.info('Cost function=' + str(partial_cost))
        SIMULATION_LOGGER.info('Simulation timeout=%.2f s' % sim_time)

        with open(os.path.join(runner.current_sim_results_dir, 'smac_result.yaml'), 'w+') as smac_file:
            yaml.dump(output, smac_file, default_flow_style=False)

        sleep(random.random())

    except Exception as e:
        SIMULATION_LOGGER.error(
            'Error occurred in this simulation evaluation, setting simulation '
            'status to CRASHED for task <%s>, message=%s' % (task, str(e)))
        status = SIM_CRASHED
        partial_cost = 1e7

        add_to_crash_log(dict(
            status=status,
            timestamp=str(datetime.datetime.now().isoformat()),
            results_dir=runner.current_sim_results_dir,
            message=str(e),
            task=str(task)))

        output = dict(
            status=status,
            cost=partial_cost,
            sim_time=None,
            message=str(e),
            task=str(task),
            results_dir=runner.current_sim_results_dir)
        PROCESS_LOCK.release()

        if runner is not None:
            if not runner.record_all_results:
                runner.remove_recording_dir()
            del runner
        if sim_eval is not None:
            del sim_eval

        return output

    if runner is not None:
        if not runner.record_all_results:
            SIMULATION_LOGGER.warning(
                'Removing recording directory, dir=' + runner.current_sim_results_dir)
            runner.remove_recording_dir()
        else:
            SIMULATION_LOGGER.warning(
                'Keeping recording directory, dir=' + runner.current_sim_results_dir)
        del runner
    if sim_eval is not None:
        del sim_eval

    PROCESS_LOCK.release()
    sleep(random.random() * 5)
    return output


def start_simulation_pool(
        max_num_processes=None,
        tasks=None,
        log_filename=None,
        output_dir=None,
        del_failed_tasks=False):
    global THREAD_POOL
    init_logger(log_filename)

    opt_config = OptConfiguration.get_instance()
    opt_config.print_params()

    num_processes = max_num_processes
    if num_processes is None:
        num_processes = opt_config.max_num_processes

    SIMULATION_LOGGER.info('Starting simulation pool, num_processes=%d' % num_processes)

    if output_dir is not None:
        original_results_path = deepcopy(opt_config.results_dir)
        opt_config.results_dir = output_dir
    else:
        original_results_path = None

    try:
        THREAD_POOL = Pool(processes=num_processes)

        task_list = tasks if tasks is not None else opt_config.tasks
        output = THREAD_POOL.map(run_simulation, task_list)
    except Exception as e:
        SIMULATION_LOGGER.error('Error! Killing all processes, message=' + str(e))
        if THREAD_POOL is not None:
            THREAD_POOL.terminate()
            THREAD_POOL.join()
            del THREAD_POOL
        THREAD_POOL = None
        return None, None
    else:
        THREAD_POOL.close()
        THREAD_POOL.join()

    if THREAD_POOL is not None:
        del THREAD_POOL
    THREAD_POOL = None

    failed_tasks = list()
    has_crashed = True
    counter = 0

    while has_crashed and counter < 3:
        failed_tasks = list()
        SIMULATION_LOGGER.warning('List of outputs=' + str(output))
        for i in range(len(output)):
            if output[i]['status'] == SIM_CRASHED:
                failed_tasks.append(i)

        if len(failed_tasks) == 0:
            has_crashed = False
            break

        SIMULATION_LOGGER.error('Some task runs have crashed, list=' + str(failed_tasks))
        SIMULATION_LOGGER.error('Rerun counter=' + str(counter))

        for i in failed_tasks:
            failed_path, failed_dir = os.path.split(output[i]['results_dir'])
            SIMULATION_LOGGER.warning('Renaming folder from failed task:')
            SIMULATION_LOGGER.warning('\t From: ' + output[i]['results_dir'])
            SIMULATION_LOGGER.warning(
                '\t To: ' + os.path.join(failed_path, 'failed_' + failed_dir))

            if os.path.isdir(output[i]['results_dir']):
                if not del_failed_tasks:
                    os.rename(
                        output[i]['results_dir'],
                        os.path.join(failed_path, 'failed_' + failed_dir))
                    SIMULATION_LOGGER.warning(
                        'Failed task directory renamed='
                        + os.path.join(failed_path, 'failed_' + failed_dir))
                else:
                    shutil.rmtree(output[i]['results_dir'])
                    SIMULATION_LOGGER.warning(
                        'Failed task directory deleted=' + output[i]['results_dir'])

            SIMULATION_LOGGER.info('Running task %d <%s>' % (i, output[i]['task']))

            try:
                THREAD_POOL = Pool(processes=1)
                output[i] = THREAD_POOL.map(run_simulation, [output[i]['task']])[0]
            except Exception as e:
                SIMULATION_LOGGER.error(
                    'Error! Killing all processes, message=' + str(e))
                if THREAD_POOL is not None:
                    THREAD_POOL.terminate()
                    THREAD_POOL.join()
                    del THREAD_POOL
                THREAD_POOL = None
                return None, None
            else:
                THREAD_POOL.close()
                THREAD_POOL.join()

            if THREAD_POOL is not None:
                del THREAD_POOL
            THREAD_POOL = None

        counter += 1

    failed_tasks = list()
    for i in range(len(output)):
        if output[i]['status'] == SIM_CRASHED:
            failed_tasks.append(output[i]['task'])

    if original_results_path is not None:
        opt_config.results_dir = original_results_path

    SIMULATION_LOGGER.info(
        'Ending simulation pool, # failed tasks=%d' % len(failed_tasks))

    return output, failed_tasks


def stop_simulation_pool():
    try:
        SIMULATION_LOGGER.warning('Killing all processes...')
        global THREAD_POOL
        THREAD_POOL.terminate()
        THREAD_POOL.join()

        if THREAD_POOL is not None:
            del THREAD_POOL
        THREAD_POOL = None
        SIMULATION_LOGGER.warning('Simulation pool terminated')
    except Exception as ex:
        SIMULATION_LOGGER.error(str(ex))
