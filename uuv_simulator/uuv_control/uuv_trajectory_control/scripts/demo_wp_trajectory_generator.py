#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt

import uuv_trajectory_generator


def run_generator(
        waypoint_set,
        interp_method):

    gen = (
        uuv_trajectory_generator
        .WPTrajectoryGenerator(
            full_dof=True
        )
    )

    gen.set_interp_method(
        interp_method
    )

    gen.init_waypoints(
        waypoint_set
    )

    dt = 0.05

    pnts = []

    gen.set_start_time(0)

    for ti in np.arange(
            -2,
            gen.get_max_time(),
            dt):

        pnts.append(
            gen.interpolate(
                ti
            )
        )

    fig = plt.figure()

    ax = fig.add_subplot(
        111,
        projection='3d'
    )

    ax.plot(

        [p.x for p in pnts],

        [p.y for p in pnts],

        [p.z for p in pnts],

        'b'

    )

    ax.plot(

        waypoint_set.x,

        waypoint_set.y,

        waypoint_set.z,

        'r.'

    )

    ax.grid(True)

    ax.set_title(
        interp_method
    )

    fig = plt.figure()

    ax = fig.add_subplot(
        211
    )

    for i in range(3):

        ax.plot(

            [p.t for p in pnts],

            [p.pos[i]
             for p in pnts]

        )

    ax.grid(True)

    ax.set_title(
        "Position"
    )

    ax = fig.add_subplot(
        212
    )

    for i in range(3):

        ax.plot(

            [p.t for p in pnts],

            [
                p.rot[i]
                *
                180
                /
                np.pi

                for p in pnts
            ]

        )

    ax.grid(True)

    ax.set_title(
        "Orientation"
    )


def main():

    wp = (
        uuv_trajectory_generator
        .WaypointSet()
    )

    wp.add_waypoint(

        uuv_trajectory_generator
        .Waypoint(
            -10,
            -12,
            -36,
            0.5
        )

    )

    wp.add_waypoint(

        uuv_trajectory_generator
        .Waypoint(
            -20,
            20,
            -5,
            0.5
        )

    )

    wp.add_waypoint(

        uuv_trajectory_generator
        .Waypoint(
            -30,
            60,
            -50,
            0.5
        )

    )

    run_generator(
        wp,
        "cubic_interpolator"
    )

    run_generator(
        wp,
        "lipb_interpolator"
    )

    plt.show()


if __name__ == "__main__":

    main()
