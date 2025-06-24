import pytest
from new_movement.new_movement.entities.States import Vector2D, State
from new_movement.new_movement.entities.Trajectory import TrajectorySegment, Trajectory
from new_movement.new_movement.entities.Motion import MotionPrimitive, MotionPath

# Física: p(t) = p0 + v0*t + 0.5*a*t^2; v(t) = v0 + a*t
def mock_integrate_t(initial_state_tuple, motion_path, t):
    px, py, vx, vy = initial_state_tuple

    time_remaining = t
    for prim in motion_path:
        duration = min(prim.duration, time_remaining)

        ax, ay = prim.acceleration.x, prim.acceleration.y

        px += vx * duration + 0.5 * ax * duration**2
        py += vy * duration + 0.5 * ay * duration**2
        vx += ax * duration
        vy += ay * duration

        time_remaining -= duration
        if time_remaining <= 1e-9:
            break

    return (px, py, vx, vy)


def mock_integrate(initial_state_tuple, motion_path):
    duration = sum(p.duration for p in motion_path)
    return mock_integrate_t(initial_state_tuple, motion_path, duration)


# Monkeypatch para substituir as funções reais pelas mocks durante os testes
@pytest.fixture(autouse=True)
def patch_integrators(monkeypatch):
    monkeypatch.setattr("new_movement.entities.Trajectory.integrate_t", mock_integrate_t)
    monkeypatch.setattr("new_movement.entities.Trajectory.integrate", mock_integrate)


@pytest.fixture
def segment1():
    """Segmento 1: Duração 2s"""
    init_pos = Vector2D(0, 0)
    init_vel = Vector2D(10, 0)
    motion = MotionPath([MotionPrimitive(acceleration=Vector2D(0, 5), duration=2.0)])
    return TrajectorySegment(init_pos, init_vel, motion)


@pytest.fixture
def segment2(segment1):
    """Segmento 2: Contínuo ao segmento 1, Duração 1s"""
    init_state = segment1.get_local_destination()
    motion = MotionPath([MotionPrimitive(acceleration=Vector2D(-10, 0), duration=1.0)])
    return TrajectorySegment(init_state.position, init_state.velocity, motion)


# --- Testes para TrajectorySegment ---


def test_segment_constructor(segment1):
    assert segment1.init_pos == Vector2D(0, 0)
    assert segment1.init_vel == Vector2D(10, 0)
    assert segment1.child is None


def test_segment_get_local_duration(segment1):
    assert segment1.get_local_duration() == 2.0


def test_segment_get_local_destination(segment1):
    dest = segment1.get_local_destination()
    # p(2) = (0,0) + (10,0)*2 + 0.5*(0,5)*2^2 = (20, 10)
    # v(2) = (10,0) + (0,5)*2 = (10, 10)
    assert dest.position.distance(Vector2D(20, 10)) < 1e-6
    assert dest.velocity.distance(Vector2D(10, 10)) < 1e-6


def test_segment_get_state_within_segment(segment1):
    state_t1 = segment1.get_state(1.0)
    # p(1) = (0,0) + (10,0)*1 + 0.5*(0,5)*1^2 = (10, 2.5)
    # v(1) = (10,0) + (0,5)*1 = (10, 5)
    assert state_t1.position.distance(Vector2D(10, 2.5)) < 1e-6
    assert state_t1.velocity.distance(Vector2D(10, 5)) < 1e-6


def test_segment_add_child_continuous(segment1, segment2):
    segment1.add_child(segment2)
    assert segment1.child == segment2


def test_segment_add_child_discontinuous(segment1):
    bad_segment = TrajectorySegment(Vector2D(99, 99), Vector2D(0, 0), MotionPath([]))
    with pytest.raises(ValueError, match="trajetória não contínuo"):
        segment1.add_child(bad_segment)


def test_segment_get_total_duration_with_child(segment1, segment2):
    segment1.add_child(segment2)
    assert segment1.get_total_duration() == 3.0  # 2s + 1s


def test_segment_get_destination_with_child(segment1, segment2):
    segment1.add_child(segment2)
    final_dest = segment1.get_destination()
    expected_dest = segment2.get_local_destination()
    assert final_dest.position == expected_dest.position
    assert final_dest.velocity == expected_dest.velocity


def test_segment_get_state_in_child_segment(segment1, segment2):
    segment1.add_child(segment2)
    # t=2.5s, que é 0.5s dentro do segundo segmento
    state = segment1.get_state(2.5)
    expected_state = segment2.get_state(0.5)
    assert state.position.distance(expected_state.position) < 1e-6
    assert state.velocity.distance(expected_state.velocity) < 1e-6


def test_segment_get_acceleration(segment1, segment2):
    segment1.add_child(segment2)
    assert segment1.get_acceleration(1.5) == Vector2D(0, 5)  # Em segment1
    assert segment1.get_acceleration(2.5) == Vector2D(-10, 0)  # Em segment2


# --- Testes para Trajectory ---


def test_trajectory_empty():
    traj = Trajectory()
    assert traj.root is None
    assert traj.tail is None
    assert traj.get_total_duration() == 0.0
    assert traj.get_state(1.0) is None
    assert traj.to_list(samples_size=10) == []


def test_trajectory_append(segment1, segment2):
    traj = Trajectory(segment1)
    assert traj.root == segment1
    assert traj.tail == segment1

    traj.append(segment2)
    assert traj.root.child == segment2
    assert traj.tail == segment2
    assert traj.get_total_duration() == 3.0


def test_trajectory_relocate(segment1, segment2):
    traj = Trajectory(segment1)
    traj.append(segment2)

    new_root_state = segment1.get_state(1.0)  # Ponto no meio do seg1
    new_root = TrajectorySegment(new_root_state.position, new_root_state.velocity, MotionPath([]))

    # Guarda o filho original para comparar depois
    original_child = traj.root.child

    traj.relocate(new_root)

    assert traj.root == new_root
    assert traj.root.child == original_child


def test_trajectory_connect(segment1, segment2):
    traj = Trajectory(segment1)
    traj.append(segment2)  # Duração total = 3.0s

    # Ponto de conexão em t=1.5s (no meio do segment1)
    connection_state = traj.get_state(1.5)
    new_motion = MotionPath([MotionPrimitive(Vector2D(1, 1), 2.0)])
    new_segment = TrajectorySegment(
        connection_state.position, connection_state.velocity, new_motion
    )

    traj.connect(new_segment, 1.5)

    # A duração do primeiro segmento deve ser agora 1.5s
    assert traj.root.get_local_duration() == 1.5
    # O filho do primeiro segmento deve ser o novo segmento
    assert traj.root.child == new_segment
    # A duração total deve ser 1.5s (antiga) + 2.0s (nova) = 3.5s
    assert traj.get_total_duration() == 3.5
    # A cauda deve ser o novo segmento
    assert traj.tail == new_segment


def test_trajectory_connect_out_of_bounds():
    traj = Trajectory(segment1)
    with pytest.raises(ValueError):
        traj.connect(segment2, 99.0)  # Tempo muito a frente
    with pytest.raises(ValueError):
        traj.connect(segment2, -1.0)  # Tempo negativo


def test_trajectory_to_list(segment1, segment2):
    traj = Trajectory(segment1)
    traj.append(segment2)  # Duração total 3.0s

    # Teste com samples_size
    path_list = traj.to_list(samples_size=4)  # Amostras em t=0, 1, 2, 3
    assert len(path_list) == 4
    assert path_list[0].distance(segment1.init_pos) < 1e-6
    assert path_list[-1].distance(traj.get_destination().position) < 1e-6

    # Teste com time_step
    path_list_states = traj.to_list(time_step=1.5, output_states=True)  # t=0, 1.5, 3.0
    assert len(path_list_states) == 3
    assert isinstance(path_list_states[0], State)
    assert path_list_states[-1].position.distance(traj.get_destination().position) < 1e-6

    # Teste de erro nos parâmetros
    with pytest.raises(ValueError):
        traj.to_list()
    with pytest.raises(ValueError):
        traj.to_list(samples_size=5, time_step=1.0)
