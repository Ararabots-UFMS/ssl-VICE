from unittest.mock import MagicMock, patch

import pytest

import gui_interpreter.apiNode as apiNode_module
from gui_interpreter.apiNode import APINode


@pytest.fixture
def node():
    instance = APINode.__new__(APINode)

    instance.publisher = MagicMock()
    instance.get_logger = MagicMock(return_value=MagicMock())

    instance.vision_running = False
    instance.vision_subscriber = None
    instance.trajectory_subscriber = MagicMock()
    instance.referee_subscriber = MagicMock()

    instance.communication_running = False
    instance.referee_running = False

    instance.robots = []
    instance.robot_count = 0

    instance.is_field_side_right = True
    instance.is_team_color_yellow = True
    instance.is_play_pressed = False
    instance.is_simulation = True

    instance.pid_client = MagicMock()
    instance.kp_angular_client = MagicMock()
    instance.set_orientation_client = MagicMock()
    instance.set_team_color_client = MagicMock()

    # Movement is a published topic now, not a service.
    instance.movement_pub = MagicMock()
    instance._movement_commands = {}
    instance.strategy_enabled_pub = MagicMock()
    instance.control_mode = apiNode_module.MANUAL_MODE

    instance.latest_geometry = None
    instance.geometry_subscriber = MagicMock()

    instance.create_subscription = MagicMock(return_value=MagicMock())
    instance.create_timer = MagicMock()
    instance.create_publisher = MagicMock()
    instance.create_client = MagicMock()
    instance.get_node_names = MagicMock(return_value=[])

    return instance


@pytest.fixture
def emit_mock():
    with patch.object(apiNode_module.gui_socket, "emit") as mock_emit:
        yield mock_emit


def _fake_state(x, y, vx, vy):
    state = MagicMock()
    state.position.x = x
    state.position.y = y
    state.velocity.x = vx
    state.velocity.y = vy
    return state


def _fake_trajectory(total_duration, states=None):
    traj = MagicMock()
    traj.get_total_duration.return_value = total_duration
    traj.to_list.return_value = states or []
    return traj


class TestHandleConnect:
    def test_subscribes_to_vision_topic(self, node):
        node.create_subscription.return_value = "subscription"

        node.handle_connect()

        assert node.vision_subscriber == "subscription"
        node.create_subscription.assert_called_once()
        _, topic, callback, _qos = node.create_subscription.call_args[0]
        assert topic == "visionTopic"
        assert callback == node.emit_vision_message


class TestEmitVisionMessage:
    def test_emits_expected_payload(self, node, emit_mock):
        msg = MagicMock()
        data = {"yellow_robots": ["y1"], "blue_robots": ["b1"], "balls": ["ball1"]}

        with patch.object(apiNode_module, "todict", return_value=data) as mock_todict:
            node.emit_vision_message(msg)

        mock_todict.assert_called_once_with(msg)
        emit_mock.assert_called_once_with(
            "vision_update", {"yellow": ["y1"], "blue": ["b1"], "balls": ["ball1"]}
        )


class TestEmitTrajectoryMessage:
    def test_skips_entries_without_segments(self, node, emit_mock):
        msg = MagicMock(
            current_trajectories=[MagicMock(segments=[])],
            time_offsets=[0.0],
            pending_trajectories=[MagicMock(segments=[])],
        )

        node.emit_trajectory_message(msg)

        emit_mock.assert_called_once_with("trajectory_update", {"trajectories": []})

    def test_builds_current_points_when_time_offset_before_duration(self, node, emit_mock):
        cur_msg = MagicMock(segments=[MagicMock()], robot_id=3)
        msg = MagicMock(
            current_trajectories=[cur_msg],
            time_offsets=[0.1],
            pending_trajectories=[MagicMock(segments=[])],
        )
        state = _fake_state(1.0, 2.0, 0.5, -0.5)
        fake_traj = _fake_trajectory(total_duration=1.0, states=[state])

        with patch.object(apiNode_module, "Trajectory") as MockTrajectory:
            MockTrajectory.from_msg.return_value = fake_traj
            node.emit_trajectory_message(msg)

        MockTrajectory.from_msg.assert_any_call(cur_msg)
        fake_traj.to_list.assert_called_once_with(
            time_step=0.2, start_time=0.1, end_time=1.0, output_states=True
        )
        entry = emit_mock.call_args[0][1]["trajectories"][0]
        assert entry["robot_id"] == 3
        assert entry["current"]["points"] == [{"x": 1.0, "y": 2.0, "vx": 0.5, "vy": -0.5}]
        assert entry["current"]["total_duration"] == 1.0
        assert entry["current"]["time_offset"] == 0.1
        assert entry["pending"]["points"] == []
        assert entry["pending"]["handoff_stamp"] is None

    def test_no_current_points_when_time_offset_past_duration(self, node, emit_mock):
        cur_msg = MagicMock(segments=[MagicMock()], robot_id=1)
        msg = MagicMock(
            current_trajectories=[cur_msg],
            time_offsets=[5.0],
            pending_trajectories=[MagicMock(segments=[])],
        )
        fake_traj = _fake_trajectory(total_duration=1.0)

        with patch.object(apiNode_module, "Trajectory") as MockTrajectory:
            MockTrajectory.from_msg.return_value = fake_traj
            node.emit_trajectory_message(msg)

        fake_traj.to_list.assert_not_called()
        entry = emit_mock.call_args[0][1]["trajectories"][0]
        assert entry["current"]["points"] == []

    def test_builds_pending_points_when_duration_significant(self, node, emit_mock):
        cur_msg = MagicMock(segments=[MagicMock()], robot_id=2)
        pending_msg = MagicMock(segments=[MagicMock()], handoff_stamp=42)
        msg = MagicMock(
            current_trajectories=[cur_msg],
            time_offsets=[0.0],
            pending_trajectories=[pending_msg],
        )
        cur_traj = _fake_trajectory(total_duration=2.0, states=[])
        pend_state = _fake_state(10, 20, 1, 2)
        pend_traj = _fake_trajectory(total_duration=0.5, states=[pend_state])

        with patch.object(apiNode_module, "Trajectory") as MockTrajectory:
            MockTrajectory.from_msg.side_effect = [cur_traj, pend_traj]
            node.emit_trajectory_message(msg)

        pend_traj.to_list.assert_called_once_with(
            time_step=0.2, start_time=0.0, end_time=0.5, output_states=True
        )
        entry = emit_mock.call_args[0][1]["trajectories"][0]
        assert entry["pending"]["points"] == [{"x": 10, "y": 20, "vx": 1, "vy": 2}]
        assert entry["pending"]["handoff_stamp"] == 42

    def test_ignores_negligible_pending_duration(self, node, emit_mock):
        cur_msg = MagicMock(segments=[MagicMock()], robot_id=2)
        pending_msg = MagicMock(segments=[MagicMock()], handoff_stamp=99)
        msg = MagicMock(
            current_trajectories=[cur_msg],
            time_offsets=[0.0],
            pending_trajectories=[pending_msg],
        )
        cur_traj = _fake_trajectory(total_duration=1.0, states=[])
        pend_traj = _fake_trajectory(total_duration=1e-4, states=[])

        with patch.object(apiNode_module, "Trajectory") as MockTrajectory:
            MockTrajectory.from_msg.side_effect = [cur_traj, pend_traj]
            node.emit_trajectory_message(msg)

        pend_traj.to_list.assert_not_called()
        entry = emit_mock.call_args[0][1]["trajectories"][0]
        assert entry["pending"]["points"] == []
        assert entry["pending"]["handoff_stamp"] is None

    def test_multiple_entries_preserve_index_alignment(self, node, emit_mock):
        empty_msg = MagicMock(segments=[])
        cur_msg = MagicMock(segments=[MagicMock()], robot_id=7)
        msg = MagicMock(
            current_trajectories=[empty_msg, cur_msg],
            time_offsets=[0.0, 0.0],
            pending_trajectories=[MagicMock(segments=[]), MagicMock(segments=[])],
        )
        fake_traj = _fake_trajectory(total_duration=1.0, states=[])

        with patch.object(apiNode_module, "Trajectory") as MockTrajectory:
            MockTrajectory.from_msg.return_value = fake_traj
            node.emit_trajectory_message(msg)

        trajectories = emit_mock.call_args[0][1]["trajectories"]
        assert len(trajectories) == 1
        assert trajectories[0]["robot_id"] == 7


class TestEmitRefereeMessage:
    def test_builds_expected_payload(self, node, emit_mock):
        msg = MagicMock()
        data = {
            "stage": "NORMAL_FIRST_HALF",
            "stage_time_left": 100,
            "command": "HALT",
            "command_counter": 5,
            "current_action_time_remaining": 3,
            "teams": [{"name": "A"}],
        }

        with patch.object(apiNode_module, "todict", return_value=data):
            node.emit_referee_message(msg)

        emit_mock.assert_called_once_with("referee_update", data)

    def test_defaults_teams_to_empty_list_and_missing_fields_to_none(self, node, emit_mock):
        msg = MagicMock()
        with patch.object(apiNode_module, "todict", return_value={"stage": "X"}):
            node.emit_referee_message(msg)

        payload = emit_mock.call_args[0][1]
        assert payload["teams"] == []
        assert payload["stage_time_left"] is None
        assert payload["command"] is None


class TestHandleDisconnect:
    def test_clears_all_subscribers(self, node):
        node.handle_disconnect()

        assert node.vision_subscriber is None
        assert node.trajectory_subscriber is None
        assert node.referee_subscriber is None


class TestSimpleStateHandlers:
    def test_handle_field_side_updates_state(self, node):
        node.handle_field_side(False)
        assert node.is_field_side_right is False

    def test_handle_team_color_updates_state(self, node):
        node.handle_team_color(False)
        assert node.is_team_color_yellow is False

    def test_handle_simulation_updates_state(self, node):
        node.handle_simulation(False)
        assert node.is_simulation is False


class TestUpdateVisionStatus:
    def test_toggles_and_emits_when_state_changes(self, node, emit_mock):
        node.vision_running = False

        node.update_vision_status(["visionNode"])

        assert node.vision_running is True
        emit_mock.assert_called_once_with("visionStatus", {"status": True})

    def test_noop_when_state_unchanged(self, node, emit_mock):
        node.vision_running = False

        node.update_vision_status(["otherNode"])

        assert node.vision_running is False
        emit_mock.assert_not_called()


class TestUpdateCommunicationStatus:
    def test_uses_sim_node_name_when_simulation(self, node, emit_mock):
        node.is_simulation = True
        node.communication_running = False

        node.update_communication_status(["grsim_publisher"])

        assert node.communication_running is True
        emit_mock.assert_called_once_with("communicationStatus", {"status": True})

    def test_uses_real_node_name_when_not_simulation(self, node, emit_mock):
        node.is_simulation = False
        node.communication_running = False

        node.update_communication_status(["hardware_publisher"])

        assert node.communication_running is True

    def test_noop_when_state_unchanged(self, node, emit_mock):
        node.is_simulation = True
        node.communication_running = False

        node.update_communication_status(["some_other_node"])

        assert node.communication_running is False
        emit_mock.assert_not_called()


class TestUpdateRefereeStatus:
    def test_toggles_and_emits_when_state_changes(self, node, emit_mock):
        node.referee_running = False

        node.update_referee_status(["refereeNode"])

        assert node.referee_running is True
        emit_mock.assert_called_once_with("refereeStatus", {"status": True})


class TestCreateMessage:
    def test_builds_message_with_robots(self, node):
        node.is_field_side_right = True
        node.is_team_color_yellow = False
        node.is_play_pressed = True
        node.robot_count = 2
        node.robots = [
            {"id": 1, "name": "R1", "address": "1,2,3", "kp": "1.5", "ki": "0.1", "kd": "0.01"},
            {"id": 2, "name": "R2", "address": "4,5,6", "kp": "2.0", "ki": "0.2", "kd": "0.02"},
        ]

        msg = node.create_message()

        assert msg.is_field_side_left is False
        assert msg.is_team_color_yellow is False
        assert msg.is_play_pressed is True
        assert msg.robot_count == 2
        assert len(msg.robots) == 2

        r1 = msg.robots[0]
        assert r1.id == 1
        assert r1.name == "R1"
        assert r1.address == [1, 2, 3]
        assert r1.kp == 1.5
        assert r1.ki == 0.1
        assert r1.kd == 0.01

    def test_with_no_robots(self, node):
        node.robots = []
        node.robot_count = 0

        msg = node.create_message()

        assert msg.robots == []
        assert msg.robot_count == 0


class TestPublishGuiData:
    def test_publishes_and_updates_all_statuses(self, node):
        node.get_node_names = MagicMock(return_value=["visionNode"])
        node.update_vision_status = MagicMock()
        node.update_communication_status = MagicMock()
        node.update_referee_status = MagicMock()

        node.publish_gui_data()

        node.publisher.publish.assert_called_once()
        published = node.publisher.publish.call_args[0][0]
        assert published.robot_count == node.robot_count
        node.update_vision_status.assert_called_once_with(["visionNode"])
        node.update_communication_status.assert_called_once_with(["visionNode"])
        node.update_referee_status.assert_called_once_with(["visionNode"])


class TestHandleConfigButton:
    def test_stores_robots_and_republishes(self, node):
        node.publish_gui_data = MagicMock()
        robots = [{"id": 1}, {"id": 2}]

        node.handle_config_button(robots)

        assert node.robot_count == 2
        assert node.robots == robots
        node.publish_gui_data.assert_called_once()


class TestHandleStrategyCommand:
    def test_publishes_movement_command(self, node, emit_mock):
        data = {
            "robot_id": "3",
            "position_x": "100.0",
            "position_y": "200.0",
            "velocity_x": "1.0",
            "velocity_y": "2.0",
        }
        node.handle_strategy_command(data)

        sent = node.movement_pub.publish.call_args[0][0]
        assert len(sent.commands) == 1
        cmd = sent.commands[0]
        assert cmd.robot_id == 3
        assert cmd.target_pos.x == 100.0
        assert cmd.target_pos.y == 200.0
        assert cmd.target_vel.x == 1.0
        assert cmd.target_vel.y == 2.0
        emit_mock.assert_called_once_with(
            "strategy_response",
            {"success": True, "message": "Command executed successfully"},
        )

    def test_defaults_velocity_to_zero_when_missing(self, node):
        node.handle_strategy_command({"robot_id": "1", "position_x": "0", "position_y": "0"})

        cmd = node.movement_pub.publish.call_args[0][0].commands[0]
        assert cmd.target_vel.x == 0.0
        assert cmd.target_vel.y == 0.0

    def test_republishes_the_whole_set(self, node):
        """The manager takes the full array each time, so earlier robots must survive."""
        node.handle_strategy_command({"robot_id": 1, "position_x": 10, "position_y": 20})
        node.handle_strategy_command({"robot_id": 2, "position_x": 30, "position_y": 40})

        sent = node.movement_pub.publish.call_args[0][0]
        assert sorted(c.robot_id for c in sent.commands) == [1, 2]

    def test_refused_in_strategy_mode(self, node, emit_mock):
        node.control_mode = apiNode_module.STRATEGY_MODE

        node.handle_strategy_command({"robot_id": 1, "position_x": 0, "position_y": 0})

        node.movement_pub.publish.assert_not_called()
        emit_mock.assert_called_once_with(
            "strategy_response",
            {
                "success": False,
                "message": "Strategy is driving; switch to manual mode to command robots",
            },
        )

    def test_emits_error_on_malformed_data(self, node, emit_mock):
        node.handle_strategy_command({})  # missing required keys -> KeyError

        emit_mock.assert_called_once()
        event, payload = emit_mock.call_args[0]
        assert event == "strategy_response"
        assert payload["success"] is False
        node.movement_pub.publish.assert_not_called()


class TestHandleControlMode:
    def test_switching_to_manual_disables_strategy(self, node, emit_mock):
        node.control_mode = apiNode_module.STRATEGY_MODE

        node.handle_control_mode({"mode": "manual"})

        assert node.control_mode == apiNode_module.MANUAL_MODE
        assert node.strategy_enabled_pub.publish.call_args[0][0].data is False
        emit_mock.assert_called_once_with("control_mode_update", {"mode": "manual"})

    def test_switching_to_strategy_enables_and_clears_commands(self, node, emit_mock):
        node.handle_strategy_command({"robot_id": 1, "position_x": 0, "position_y": 0})
        emit_mock.reset_mock()

        node.handle_control_mode({"mode": "strategy"})

        assert node.control_mode == apiNode_module.STRATEGY_MODE
        assert node.strategy_enabled_pub.publish.call_args[0][0].data is True
        assert node._movement_commands == {}
        # Publishing an empty array would strip every robot of its target for one tick.
        node.movement_pub.publish.assert_called_once()
        emit_mock.assert_called_once_with("control_mode_update", {"mode": "strategy"})

    def test_switching_to_manual_parks_the_given_robots(self, node):
        """Strategy's last goals stay latched in the manager, so they must be overridden."""
        node.control_mode = apiNode_module.STRATEGY_MODE

        node.handle_control_mode({"mode": "manual", "robots": [
            {"robot_id": 1, "position_x": 100.0, "position_y": -200.0},
            {"robot_id": 2, "position_x": 0.0, "position_y": 50.0},
        ]})

        sent = node.movement_pub.publish.call_args[0][0]
        by_id = {c.robot_id: c for c in sent.commands}
        assert by_id[1].target_pos.x == 100.0
        assert by_id[1].target_vel.x == 0.0
        assert by_id[2].target_pos.y == 50.0

    def test_switching_to_manual_without_robots_publishes_nothing(self, node):
        node.control_mode = apiNode_module.STRATEGY_MODE

        node.handle_control_mode({"mode": "manual"})

        node.movement_pub.publish.assert_not_called()

    def test_rejects_unknown_mode(self, node, emit_mock):
        node.control_mode = apiNode_module.MANUAL_MODE

        node.handle_control_mode({"mode": "banana"})

        assert node.control_mode == apiNode_module.MANUAL_MODE
        node.strategy_enabled_pub.publish.assert_not_called()
        event, payload = emit_mock.call_args[0]
        assert event == "control_mode_update"
        assert payload["mode"] == "manual"
        assert "error" in payload


class TestHandleStopRobots:
    def test_parks_each_robot_at_its_current_position(self, node, emit_mock):
        node.handle_stop_robots({"robots": [
            {"robot_id": 1, "position_x": 100.0, "position_y": -200.0},
            {"robot_id": 2, "position_x": 0.0, "position_y": 50.0},
        ]})

        sent = node.movement_pub.publish.call_args[0][0]
        by_id = {c.robot_id: c for c in sent.commands}
        assert by_id[1].target_pos.x == 100.0
        assert by_id[1].target_pos.y == -200.0
        assert by_id[1].target_vel.x == 0.0
        assert by_id[1].target_vel.y == 0.0
        assert by_id[2].target_pos.y == 50.0
        emit_mock.assert_called_once_with(
            "strategy_response", {"success": True, "message": "Stopped 2 robot(s)"}
        )

    def test_overwrites_a_pending_target(self, node):
        node.handle_strategy_command({"robot_id": 1, "position_x": 3000, "position_y": 0})

        node.handle_stop_robots({"robots": [{"robot_id": 1, "position_x": 10, "position_y": 20}]})

        cmd = node.movement_pub.publish.call_args[0][0].commands[0]
        assert cmd.target_pos.x == 10.0

    def test_refused_in_strategy_mode(self, node, emit_mock):
        node.control_mode = apiNode_module.STRATEGY_MODE

        node.handle_stop_robots({"robots": [{"robot_id": 1, "position_x": 0, "position_y": 0}]})

        node.movement_pub.publish.assert_not_called()
        _, payload = emit_mock.call_args[0]
        assert payload["success"] is False

    def test_emits_error_on_malformed_data(self, node, emit_mock):
        node.handle_stop_robots({"robots": [{"robot_id": 1}]})  # missing position

        node.movement_pub.publish.assert_not_called()
        _, payload = emit_mock.call_args[0]
        assert payload["success"] is False


class TestEmitGeometryMessage:
    def test_caches_and_emits(self, node, emit_mock):
        msg = MagicMock()

        with patch.object(apiNode_module, "todict", return_value={"field_length": 9000}):
            node.emit_geometry_message(msg)

        assert node.latest_geometry == {"field_length": 9000}
        emit_mock.assert_called_once_with("geometry_update", {"field_length": 9000})

class TestUpdatePidAndKpAngularAndOrientationRequests:
    def test_handle_update_pid_sends_request(self, node):
        node.pid_client.wait_for_service.return_value = True
        future = MagicMock()
        node.pid_client.call_async.return_value = future

        node.handle_update_pid({"robot_id": "5", "kp": "1.1", "ki": "2.2", "kd": "3.3"})

        req = node.pid_client.call_async.call_args[0][0]
        assert req.id == 5
        assert req.kp == 1.1
        assert req.ki == 2.2
        assert req.kd == 3.3
        future.add_done_callback.assert_called_once_with(node.handle_pid_response)

    def test_handle_update_kp_angular_sends_request(self, node):
        node.kp_angular_client.wait_for_service.return_value = True
        node.kp_angular_client.call_async.return_value = MagicMock()

        node.handle_update_kp_angular({"kp": "9.9"})

        req = node.kp_angular_client.call_async.call_args[0][0]
        assert req.kp == 9.9

    def test_handle_set_orientation_sends_request(self, node):
        node.set_orientation_client.wait_for_service.return_value = True
        node.set_orientation_client.call_async.return_value = MagicMock()

        node.handle_set_orientation({"robot_id": "7", "orientation": "1.57"})

        req = node.set_orientation_client.call_async.call_args[0][0]
        assert req.robot_id == 7
        assert req.orientation == 1.57


class TestHandleUpdateObstacles:
    def test_sets_planning_options(self, node, emit_mock):
        data = {
            "robot_id": "1",
            "penalty_area": False,
            "center_area": True,
            "ball": False,
        }
        node.handle_update_obstacles(data)

        cmd = node.movement_pub.publish.call_args[0][0].commands[0]
        assert cmd.robot_id == 1
        assert cmd.planning_options.avoid_penalty_area is False
        assert cmd.planning_options.avoid_center_area is True
        assert cmd.planning_options.avoid_ball is False
        emit_mock.assert_called_once_with(
            "obstacles_response",
            {"success": True, "message": "Obstacles updated successfully"},
        )

    def test_defaults_missing_fields_to_false(self, node):
        node.handle_update_obstacles({"robot_id": "1"})

        options = node.movement_pub.publish.call_args[0][0].commands[0].planning_options
        assert options.avoid_penalty_area is False
        assert options.avoid_center_area is False
        assert options.avoid_ball is False

    def test_shares_the_command_with_the_position_handler(self, node):
        """Both panels write the same MovementCommand, so neither may clear the other."""
        node.handle_strategy_command({"robot_id": 1, "position_x": 500, "position_y": 600})
        node.handle_update_obstacles({"robot_id": 1, "ball": True})

        sent = node.movement_pub.publish.call_args[0][0]
        assert len(sent.commands) == 1
        cmd = sent.commands[0]
        assert cmd.target_pos.x == 500.0
        assert cmd.planning_options.avoid_ball is True

    def test_refused_in_strategy_mode(self, node, emit_mock):
        node.control_mode = apiNode_module.STRATEGY_MODE

        node.handle_update_obstacles({"robot_id": "1"})

        node.movement_pub.publish.assert_not_called()
        _, payload = emit_mock.call_args[0]
        assert payload["success"] is False

    def test_emits_error_on_malformed_data(self, node, emit_mock):
        node.handle_update_obstacles({})  # missing robot_id -> KeyError

        node.movement_pub.publish.assert_not_called()
        _, payload = emit_mock.call_args[0]
        assert payload["success"] is False


class TestHandleTeamColorService:
    def test_sends_request_with_true(self, node):
        node.set_team_color_client.wait_for_service.return_value = True
        node.set_team_color_client.call_async.return_value = MagicMock()

        node.handle_team_color_service({"is_yellow": True})

        req = node.set_team_color_client.call_async.call_args[0][0]
        assert req.data is True

    def test_defaults_to_false_when_missing(self, node):
        node.set_team_color_client.wait_for_service.return_value = True
        node.set_team_color_client.call_async.return_value = MagicMock()

        node.handle_team_color_service({})

        req = node.set_team_color_client.call_async.call_args[0][0]
        assert req.data is False


@pytest.mark.parametrize(
    "handler_name,client_attr,event,message",
    [
        ("handle_update_pid", "pid_client", "pid_response", "PID service is not available"),
        (
            "handle_update_kp_angular",
            "kp_angular_client",
            "kp_angular_response",
            "Kp angular service is not available",
        ),
        (
            "handle_set_orientation",
            "set_orientation_client",
            "orientation_response",
            "Set orientation service is not available",
        ),
        (
            "handle_team_color_service",
            "set_team_color_client",
            "team_color_response",
            "Set team color service is not available",
        ),
    ],
)
def test_handler_emits_error_when_service_unavailable(
    node, emit_mock, handler_name, client_attr, event, message
):
    getattr(node, client_attr).wait_for_service.return_value = False

    getattr(node, handler_name)({})

    emit_mock.assert_called_once_with(event, {"success": False, "message": message})
    getattr(node, client_attr).call_async.assert_not_called()


@pytest.mark.parametrize(
    "handler_name,event,success_msg,failure_msg,error_msg",
    [
        (
            "handle_pid_response",
            "pid_response",
            "PID updated successfully",
            "PID update failed",
            "PID service call failed",
        ),
        (
            "handle_kp_angular_response",
            "kp_angular_response",
            "Kp angular updated successfully",
            "Kp angular update failed",
            "Kp angular service call failed",
        ),
        (
            "handle_orientation_response",
            "orientation_response",
            "Orientation set successfully",
            "Set orientation failed",
            "Set orientation service call failed",
        ),
        (
            "handle_team_color_service_response",
            "team_color_response",
            "Team color set successfully",
            "Set team color failed",
            "Set team color service call failed",
        ),
    ],
)
class TestGenericResponseHandlers:
    def test_success(self, node, emit_mock, handler_name, event, success_msg, failure_msg, error_msg):
        future = MagicMock()
        future.result.return_value = MagicMock(success=True)

        getattr(node, handler_name)(future)

        emit_mock.assert_called_once_with(event, {"success": True, "message": success_msg})

    def test_failure(self, node, emit_mock, handler_name, event, success_msg, failure_msg, error_msg):
        future = MagicMock()
        future.result.return_value = MagicMock(success=False)

        getattr(node, handler_name)(future)

        emit_mock.assert_called_once_with(event, {"success": False, "message": failure_msg})

    def test_exception(self, node, emit_mock, handler_name, event, success_msg, failure_msg, error_msg):
        future = MagicMock()
        future.result.side_effect = RuntimeError("boom")

        getattr(node, handler_name)(future)

        _, payload = emit_mock.call_args[0]
        assert payload["success"] is False
        assert error_msg in payload["message"]


class TestCheckStrategyServicesStatus:
    def test_emits_and_returns_status_dict(self, node, emit_mock):
        # strategy and obstacles are published topics now, so they have no server to
        # probe and are reported as always available.
        node.pid_client.service_is_ready.return_value = False
        node.kp_angular_client.service_is_ready.return_value = True
        node.set_orientation_client.service_is_ready.return_value = False
        node.set_team_color_client.service_is_ready.return_value = False

        result = node.check_strategy_services_status()

        expected = {
            "strategy": True,
            "pid": False,
            "kp_angular": True,
            "orientation": False,
            "obstacles": True,
            "team_color": False,
        }
        assert result == expected
        emit_mock.assert_called_once_with("services_status", expected)
