import pytest
from new_movement.new_movement.entities.States import Vector2D
from new_movement.new_movement.entities.States import State
from new_movement.new_movement.entities.States import MoveConstraints


def test_constructor_vector2D():
    vector = Vector2D(2.0, 4.0)

    assert vector.x == 2.0
    assert vector.y == 4.0


def test_constructor_vector2D_with_0():
    vector = Vector2D(0.0, 0.0)

    assert vector.x == 0.0
    assert vector.y == 0.0


def test_neg_vector2D():
    vector = Vector2D(2.0, 4.0)
    vector = -vector

    assert vector.x == -2.0
    assert vector.y == -4.0


def test_add_vector2D():
    vector1 = Vector2D(2.0, 4.0)
    vector2 = Vector2D(6.0, 8.0)
    vector = vector1 + vector2

    assert vector == [2.0, 4.0, 6.0, 8.0]


def test_getitem_vector2D():
    vector = Vector2D(2.0, 4.0)

    assert vector.x == vector[0]
    assert vector.y == vector[1]


def test_getitem_vector2D_IndexError():
    vector = Vector2D(2.0, 4.0)
    with pytest.raises(IndexError):
        x = vector[3]
        y = vector[4]


def test_constructor_State():
    position = Vector2D(10.0, 20.0)
    velocity = Vector2D(2.0, 4.0)
    acceleration = Vector2D(1.0, 2.0)
    state = State(position, velocity, acceleration)

    assert state.position[0] == 10
    assert state.position[1] == 20
    assert state.velocity[0] == 2
    assert state.velocity[1] == 4
    assert state.acceleration[0] == 1
    assert state.acceleration[1] == 2


def test_constructor_State_aceleration_none():
    position = Vector2D(10.0, 20.0)
    velocity = Vector2D(2.0, 4.0)
    state = State(position, velocity)

    assert state.position[0] == 10
    assert state.position[1] == 20
    assert state.velocity[0] == 2
    assert state.velocity[1] == 4
    assert state.acceleration is None


def test_constructor_MoveConstraint():
    max_velocity = Vector2D(10.0, 20.0)
    max_acceleration = Vector2D(2.0, 4.0)
    move_constraint = MoveConstraints(max_velocity, max_acceleration)

    assert move_constraint.max_velocity[0] == 10
    assert move_constraint.max_velocity[1] == 20
    assert move_constraint.max_acceleration[0] == 2
    assert move_constraint.max_acceleration[1] == 4
    assert move_constraint.min_velocity[0] == -move_constraint.max_velocity[0]
    assert move_constraint.min_velocity[1] == -move_constraint.max_velocity[1]
    assert move_constraint.min_acceleration[0] == -move_constraint.max_acceleration[0]
    assert move_constraint.min_acceleration[1] == -move_constraint.max_acceleration[1]

    # A função post_init já esta sendo aplicada quando é None.
