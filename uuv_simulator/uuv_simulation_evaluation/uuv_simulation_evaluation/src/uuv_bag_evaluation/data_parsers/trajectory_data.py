# trajectory_data.py
#
# ROS2 + Gazebo Harmonic + gz-sim8 version
#
# Compatible with:
# - ROS2 Humble / Jazzy
# - rosbag2_py
# - Gazebo Harmonic
# - ros_gz_bridge
# - gz-sim8

from __future__ import annotations

import os
import numpy as np
import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d import Axes3D

from simulation_data import (
    SimulationData,
    COLOR_RED,
    COLOR_GREEN,
    COLOR_BLUE
)

from uuv_trajectory_generator import (
    TrajectoryGenerator,
    TrajectoryPoint
)

try:
    plt.rc('text', usetex=True)
    plt.rc('font', family='sans-serif')
except Exception as e:
    print(f'Cannot use LaTeX in matplotlib: {e}')


class TrajectoryData(SimulationData):

    LABEL = 'trajectory'

    def __init__(self, bag):

        super().__init__(
            message_type='nav_msgs/msg/Odometry'
        )

        self._topic_name = {}

        self._recorded_data = {
            'desired': None,
            'actual': None
        }

        try:
            topics = bag.get_topic_names_and_types()

            # ----------------------------------------------------------
            # FIND TOPICS
            # ----------------------------------------------------------
            for topic_name, topic_types in topics.items():

                msg_type = topic_types[0]

                # ODOMETRY
                if (
                    'nav_msgs/msg/Odometry' in msg_type
                ):
                    self._topic_name['odometry'] = topic_name

                    self._logger.info(
                        f'Odometry topic found <{topic_name}>'
                    )

                # REFERENCE TRAJECTORY
                if (
                    'reference' in topic_name and
                    'uuv_control_msgs/msg/TrajectoryPoint'
                    in msg_type
                ):
                    self._topic_name['reference'] = topic_name

                    self._logger.info(
                        f'Reference trajectory topic found '
                        f'<{topic_name}>'
                    )

            # ----------------------------------------------------------
            # LOAD DESIRED TRAJECTORY
            # ----------------------------------------------------------
            if 'reference' in self._topic_name:

                self._recorded_data['desired'] = (
                    TrajectoryGenerator()
                )

                messages = bag.read_messages(
                    self._topic_name['reference']
                )

                for msg, t in messages:

                    self._recorded_data[
                        'desired'
                    ].add_trajectory_point_from_msg(msg)

                self._logger.info(
                    f"{self._topic_name['reference']}=loaded"
                )

            else:
                self._logger.warning(
                    'Reference trajectory topic not found'
                )

            # ----------------------------------------------------------
            # LOAD ACTUAL ODOMETRY
            # ----------------------------------------------------------
            if 'odometry' in self._topic_name:

                self._recorded_data['actual'] = (
                    TrajectoryGenerator()
                )

                messages = bag.read_messages(
                    self._topic_name['odometry']
                )

                for msg, t in messages:

                    stamp = bag.get_time_in_seconds(msg)

                    p = msg.pose.pose.position
                    q = msg.pose.pose.orientation

                    v = msg.twist.twist.linear
                    w = msg.twist.twist.angular

                    point = TrajectoryPoint(
                        stamp,

                        np.array([
                            p.x,
                            p.y,
                            p.z
                        ]),

                        np.array([
                            q.x,
                            q.y,
                            q.z,
                            q.w
                        ]),

                        np.array([
                            v.x,
                            v.y,
                            v.z,
                            w.x,
                            w.y,
                            w.z
                        ]),

                        np.array([
                            0.0,
                            0.0,
                            0.0
                        ]),

                        np.array([
                            0.0,
                            0.0,
                            0.0
                        ]),

                        np.array([
                            0.0,
                            0.0,
                            0.0
                        ])
                    )

                    self._recorded_data[
                        'actual'
                    ].add_trajectory_point(point)

                self._logger.info(
                    f"{self._topic_name['odometry']}=loaded"
                )

            else:
                self._logger.warning(
                    'Odometry topic not found'
                )

        except Exception as e:
            self._logger.error(
                f'Error loading trajectory data: {e}'
            )

    # ==============================================================
    # PROPERTIES
    # ==============================================================

    @property
    def start_time(self):

        if (
            self._recorded_data['desired'] is None or
            len(self._recorded_data['desired'].points) == 0
        ):
            return None

        return self._recorded_data['desired'].points[0].t

    @property
    def end_time(self):

        if (
            self._recorded_data['desired'] is None or
            len(self._recorded_data['desired'].points) == 0
        ):
            return None

        return self._recorded_data['desired'].points[-1].t

    @property
    def reference(self):
        return self._recorded_data['desired']

    @property
    def odometry(self):
        return self._recorded_data['actual']

    # ==============================================================
    # DATAFRAME EXPORT
    # ==============================================================

    def get_as_dataframe(self, add_group_name=None):

        try:
            import pandas as pd

            # ------------------------------------------------------
            # REFERENCE DATAFRAME
            # ------------------------------------------------------
            ref = self._recorded_data['desired']

            data_ref = {
                f'{self.LABEL}_ref_time': ref.time
            }

            for i, tag in zip(range(3), ['x', 'y', 'z']):

                data_ref[
                    f'{self.LABEL}_pos_ref_{tag}'
                ] = [e.p[i] for e in ref.points]

                data_ref[
                    f'{self.LABEL}_lin_vel_ref_{tag}'
                ] = [e.vel[i] for e in ref.points]

                data_ref[
                    f'{self.LABEL}_ang_vel_ref_{tag}'
                ] = [e.vel[i + 3] for e in ref.points]

            for i, tag in zip(
                range(3),
                ['roll', 'pitch', 'yaw']
            ):

                data_ref[
                    f'{self.LABEL}_rot_ref_{tag}'
                ] = [e.rot[i] for e in ref.points]

            for i, tag in zip(
                range(4),
                ['x', 'y', 'z', 'w']
            ):

                data_ref[
                    f'{self.LABEL}_rotq_ref_{tag}'
                ] = [e.rotq[i] for e in ref.points]

            if add_group_name is not None:
                data_ref['group'] = [
                    add_group_name
                    for _ in range(len(ref.points))
                ]

            df_ref = pd.DataFrame(data_ref)

            # ------------------------------------------------------
            # ACTUAL DATAFRAME
            # ------------------------------------------------------
            actual = self._recorded_data['actual']

            data_actual = {
                f'{self.LABEL}_actual_time': actual.time
            }

            for i, tag in zip(range(3), ['x', 'y', 'z']):

                data_actual[
                    f'{self.LABEL}_pos_actual_{tag}'
                ] = [e.p[i] for e in actual.points]

                data_actual[
                    f'{self.LABEL}_lin_vel_actual_{tag}'
                ] = [e.vel[i] for e in actual.points]

                data_actual[
                    f'{self.LABEL}_ang_vel_actual_{tag}'
                ] = [e.vel[i + 3] for e in actual.points]

            for i, tag in zip(
                range(3),
                ['roll', 'pitch', 'yaw']
            ):

                data_actual[
                    f'{self.LABEL}_rot_actual_{tag}'
                ] = [e.rot[i] for e in actual.points]

            for i, tag in zip(
                range(4),
                ['x', 'y', 'z', 'w']
            ):

                data_actual[
                    f'{self.LABEL}_rotq_actual_{tag}'
                ] = [e.rotq[i] for e in actual.points]

            if add_group_name is not None:
                data_actual['group'] = [
                    add_group_name
                    for _ in range(len(actual.points))
                ]

            df_actual = pd.DataFrame(data_actual)

            return {
                'ref': df_ref,
                'actual': df_actual
            }

        except Exception as ex:
            self._logger.error(
                f'Error exporting dataframe: {ex}'
            )
            return None

    # ==============================================================
    # PLOTTING
    # ==============================================================

    def plot(self, output_dir):

        if not os.path.isdir(output_dir):
            raise RuntimeError(
                'Invalid output directory'
            )

        desired = self._recorded_data['desired']
        actual = self._recorded_data['actual']

        if desired is None or actual is None:
            self._logger.warning(
                'Missing trajectory data'
            )
            return

        # ==========================================================
        # 3D PATH
        # ==========================================================
        try:

            fig = plt.figure(
                figsize=(
                    self._plot_configs['figsize'][0],
                    self._plot_configs['figsize'][1]
                )
            )

            ax = fig.add_subplot(
                111,
                projection='3d'
            )

            ax.plot(
                [e.p[0] for e in desired.points],
                [e.p[1] for e in desired.points],
                [e.p[2] for e in desired.points],
                color=COLOR_BLUE,
                linestyle='dashed',
                linewidth=self._plot_configs['linewidth'],
                label='Reference Path'
            )

            ax.plot(
                [e.p[0] for e in actual.points],
                [e.p[1] for e in actual.points],
                [e.p[2] for e in actual.points],
                color=COLOR_GREEN,
                linewidth=self._plot_configs['linewidth'],
                label='Actual Path'
            )

            ax.scatter(
                actual.points[0].p[0],
                actual.points[0].p[1],
                actual.points[0].p[2],
                color=COLOR_RED,
                s=80,
                label='Start'
            )

            ax.set_xlabel('X [m]')
            ax.set_ylabel('Y [m]')
            ax.set_zlabel('Z [m]')

            ax.legend()

            ax.grid(True)

            ax.view_init(
                elev=15,
                azim=30
            )

            plt.tight_layout()

            plt.savefig(
                os.path.join(
                    output_dir,
                    'paths.pdf'
                )
            )

            plt.close(fig)

        except Exception as e:
            self._logger.error(
                f'Error plotting 3D path: {e}'
            )

        # ==========================================================
        # POSITION
        # ==========================================================
        try:

            fig = self.get_figure()

            ax = fig.gca()

            labels = ['X', 'Y', 'Z']
            colors = [
                COLOR_RED,
                COLOR_GREEN,
                COLOR_BLUE
            ]

            for i in range(3):

                ax.plot(
                    desired.time,
                    [e.p[i] for e in desired.points],
                    linestyle='dashed',
                    color=colors[i],
                    linewidth=self._plot_configs['linewidth'],
                    label=f'{labels[i]}d'
                )

                ax.plot(
                    actual.time,
                    [e.p[i] for e in actual.points],
                    color=colors[i],
                    linewidth=self._plot_configs['linewidth'],
                    label=labels[i]
                )

            self.config_2dplot(
                ax=ax,
                title='',
                xlabel='Time [s]',
                ylabel='Position [m]',
                legend_on=True
            )

            plt.tight_layout()

            plt.savefig(
                os.path.join(
                    output_dir,
                    'trajectories_position.pdf'
                )
            )

            plt.close(fig)

        except Exception as e:
            self._logger.error(
                f'Error plotting positions: {e}'
            )

        # ==========================================================
        # ORIENTATION
        # ==========================================================
        try:

            fig = self.get_figure()

            ax = fig.gca()

            labels = ['Roll', 'Pitch', 'Yaw']
            colors = [
                COLOR_RED,
                COLOR_GREEN,
                COLOR_BLUE
            ]

            for i in range(3):

                ax.plot(
                    desired.time,
                    [e.rot[i] for e in desired.points],
                    linestyle='dashed',
                    color=colors[i],
                    linewidth=self._plot_configs['linewidth'],
                    label=f'{labels[i]}d'
                )

                ax.plot(
                    actual.time,
                    [e.rot[i] for e in actual.points],
                    color=colors[i],
                    linewidth=self._plot_configs['linewidth'],
                    label=labels[i]
                )

            self.config_2dplot(
                ax=ax,
                title='',
                xlabel='Time [s]',
                ylabel='Angles [rad]',
                legend_on=True
            )

            plt.tight_layout()

            plt.savefig(
                os.path.join(
                    output_dir,
                    'trajectories_orientation.pdf'
                )
            )

            plt.close(fig)

        except Exception as e:
            self._logger.error(
                f'Error plotting orientation: {e}'
            )

        # ==========================================================
        # LINEAR VELOCITIES
        # ==========================================================
        try:

            fig = self.get_figure()

            ax = fig.gca()

            labels = ['Vx', 'Vy', 'Vz']
            colors = [
                COLOR_RED,
                COLOR_GREEN,
                COLOR_BLUE
            ]

            for i in range(3):

                ax.plot(
                    desired.time,
                    [e.vel[i] for e in desired.points],
                    linestyle='dashed',
                    color=colors[i],
                    linewidth=self._plot_configs['linewidth'],
                    label=f'{labels[i]}d'
                )

                ax.plot(
                    actual.time,
                    [e.vel[i] for e in actual.points],
                    color=colors[i],
                    linewidth=self._plot_configs['linewidth'],
                    label=labels[i]
                )

            self.config_2dplot(
                ax=ax,
                title='',
                xlabel='Time [s]',
                ylabel='Linear Velocity [m/s]',
                legend_on=True
            )

            plt.tight_layout()

            plt.savefig(
                os.path.join(
                    output_dir,
                    'trajectories_lin_vel.pdf'
                )
            )

            plt.close(fig)

        except Exception as e:
            self._logger.error(
                f'Error plotting linear velocities: {e}'
            )

        # ==========================================================
        # ANGULAR VELOCITIES
        # ==========================================================
        try:

            fig = self.get_figure()

            ax = fig.gca()

            labels = ['Wx', 'Wy', 'Wz']
            colors = [
                COLOR_RED,
                COLOR_GREEN,
                COLOR_BLUE
            ]

            for i in range(3):

                ax.plot(
                    desired.time,
                    [e.vel[i + 3] for e in desired.points],
                    linestyle='dashed',
                    color=colors[i],
                    linewidth=self._plot_configs['linewidth'],
                    label=f'{labels[i]}d'
                )

                ax.plot(
                    actual.time,
                    [e.vel[i + 3] for e in actual.points],
                    color=colors[i],
                    linewidth=self._plot_configs['linewidth'],
                    label=labels[i]
                )

            self.config_2dplot(
                ax=ax,
                title='',
                xlabel='Time [s]',
                ylabel='Angular Velocity [rad/s]',
                legend_on=True
            )

            plt.tight_layout()

            plt.savefig(
                os.path.join(
                    output_dir,
                    'trajectories_ang_vel.pdf'
                )
            )

            plt.close(fig)

        except Exception as e:
            self._logger.error(
                f'Error plotting angular velocities: {e}'
            )
