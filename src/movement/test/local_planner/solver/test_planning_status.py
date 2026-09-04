from movement.local_planner.solver import PlanningStatus


class TestPlanningStatus:
    def test_all_expected_members_present(self):
        names = {member.name for member in PlanningStatus}
        assert names == {"SUCCESS", "DIRECT_PATH", "BYPASS_FOUND", "FAILED", "RECOVERY"}

    def test_members_have_unique_values(self):
        values = [member.value for member in PlanningStatus]
        assert len(values) == len(set(values))

    def test_members_are_accessible_by_name(self):
        assert PlanningStatus["DIRECT_PATH"] is PlanningStatus.DIRECT_PATH
        assert PlanningStatus["RECOVERY"] is PlanningStatus.RECOVERY
