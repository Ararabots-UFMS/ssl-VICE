import pytest

from movement.local_planner.solver import BaseSolver, BypassSolver


class TestBaseSolver:
    def test_cannot_instantiate_directly(self):
        with pytest.raises(TypeError):
            BaseSolver()

    def test_subclass_missing_solve_cannot_instantiate(self):
        class IncompleteSolver(BaseSolver):
            pass

        with pytest.raises(TypeError):
            IncompleteSolver()

    def test_concrete_subclass_is_a_base_solver(self, sampler):
        solver = BypassSolver(max_iterations=1, sampler=sampler, collision_time_step=0.05)
        assert isinstance(solver, BaseSolver)
