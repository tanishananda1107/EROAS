#!/usr/bin/env python3

import pytest

from uuv_waypoints import Waypoint
from uuv_waypoints import WaypointSet


def test_init():
    wp_set = WaypointSet()

    assert wp_set.num_waypoints == 0


def test_invalid_params_helix():
    wp_set = WaypointSet()

    result = wp_set.generate_helix(
        radius=-1.0,
        center=None,
        num_points=-1,
        max_forward_speed=0.0,
        delta_z=1,
        num_turns=-1,
        theta_offset=0.0,
        heading_offset=0.0
    )

    assert result is False


def test_invalid_params_circle():
    wp_set = WaypointSet()

    result = wp_set.generate_circle(
        radius=-1,
        center=None,
        num_points=-1,
        max_forward_speed=0,
        theta_offset=0.0,
        heading_offset=0.0
    )

    assert result is False


def test_add_repeated_waypoint():
    wp = Waypoint(
        x=1,
        y=2,
        z=3,
        max_forward_speed=1
    )

    wp_set = WaypointSet()

    assert wp_set.add_waypoint(wp)

    assert not wp_set.add_waypoint(wp)
