from unittest.mock import MagicMock, patch

import pytest

from new_movement.utils.trajectory_plotter import TrajectoryPlotter
from utils.math_util import Vector2D


@pytest.fixture
def plotter():
    return TrajectoryPlotter()


@pytest.fixture
def fake_trajectory():
    trajectory = MagicMock()
    trajectory.to_list.return_value = [
        Vector2D(0.0, 0.0),
        Vector2D(1.0, 2.0),
        Vector2D(2.0, 4.0),
    ]
    return trajectory


class TestPlot:
    def test_requests_positions_with_expected_time_step(self, plotter, fake_trajectory):
        with patch("new_movement.utils.trajectory_plotter.plt") as plt:
            plotter.plot(fake_trajectory)
        fake_trajectory.to_list.assert_called_once_with(time_step=0.01)

    def test_plots_extracted_xy_coordinates(self, plotter, fake_trajectory):
        with patch("new_movement.utils.trajectory_plotter.plt") as plt:
            plotter.plot(fake_trajectory)
        args, kwargs = plt.plot.call_args
        xs, ys = args
        assert xs == [0.0, 1.0, 2.0]
        assert ys == [0.0, 2.0, 4.0]

    def test_uses_label_and_color(self, plotter, fake_trajectory):
        with patch("new_movement.utils.trajectory_plotter.plt") as plt:
            plotter.plot(fake_trajectory, label="MyPath", color="red")
        _, kwargs = plt.plot.call_args
        assert kwargs["label"] == "MyPath"
        assert kwargs["color"] == "red"

    def test_default_label_and_color(self, plotter, fake_trajectory):
        with patch("new_movement.utils.trajectory_plotter.plt") as plt:
            plotter.plot(fake_trajectory)
        _, kwargs = plt.plot.call_args
        assert kwargs["label"] == "Trajectory"
        assert kwargs["color"] == "blue"

    def test_show_true_calls_plt_show(self, plotter, fake_trajectory):
        with patch("new_movement.utils.trajectory_plotter.plt") as plt:
            plotter.plot(fake_trajectory, show=True)
        plt.show.assert_called_once()

    def test_show_false_does_not_call_plt_show(self, plotter, fake_trajectory):
        with patch("new_movement.utils.trajectory_plotter.plt") as plt:
            plotter.plot(fake_trajectory, show=False)
        plt.show.assert_not_called()

    def test_sets_axis_labels_and_title(self, plotter, fake_trajectory):
        with patch("new_movement.utils.trajectory_plotter.plt") as plt:
            plotter.plot(fake_trajectory)
        plt.xlabel.assert_called_once_with("X")
        plt.ylabel.assert_called_once_with("Y")
        plt.title.assert_called_once_with("Trajectory Plot")

    def test_sets_field_limits(self, plotter, fake_trajectory):
        with patch("new_movement.utils.trajectory_plotter.plt") as plt:
            plotter.plot(fake_trajectory)
        plt.xlim.assert_called_once_with(-2250, 2250)
        plt.ylim.assert_called_once_with(-1500, 1500)

    def test_enables_legend_and_grid(self, plotter, fake_trajectory):
        with patch("new_movement.utils.trajectory_plotter.plt") as plt:
            plotter.plot(fake_trajectory)
        plt.legend.assert_called_once()
        plt.grid.assert_called_once_with(True)

    def test_empty_trajectory_produces_empty_series(self, plotter):
        trajectory = MagicMock()
        trajectory.to_list.return_value = []
        with patch("new_movement.utils.trajectory_plotter.plt") as plt:
            plotter.plot(trajectory)
        args, kwargs = plt.plot.call_args
        xs, ys = args
        assert xs == []
        assert ys == []
