#!/usr/bin/env python3

import pytest

from uuv_waypoints import Waypoint


def test_equal_waypoints():
    wp0 = Waypoint(x=1, y=2, z=3)
    wp1 = Waypoint(x=1, y=2, z=3)

    assert wp0 == wp1


def test_unequal_waypoints():
    wp0 = Waypoint(x=1, y=2, z=3)
    wp1 = Waypoint(x=6, y=5, z=4)

    assert wp0 != wp1


def test_violate_constraint_flag():
    wp = Waypoint()

    wp.violates_constraint = True

    assert wp.violates_constraint


def test_distance_calculation():
    wp = Waypoint(x=0, y=2, z=0)

    assert wp.dist([0, 4, 0]) == 2
    assert wp.dist([2, 2, 0]) == 2
    assert wp.dist([0, 2, 2]) == 2


def test_to_message():
    wp0 = Waypoint(x=1, y=2, z=3)

    wp1 = Waypoint()

    wp1.from_message(wp0.to_message())

    assert wp0 == wp1
