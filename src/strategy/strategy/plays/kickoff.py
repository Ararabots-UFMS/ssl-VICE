from strategy.behaviour import LeafNode, RunResult, Selector, Sequence
from strategy.commons.check_state import CheckState
from strategy.commons.task_status import TaskStatus
from strategy.context import TickContext, TreeDeps
from strategy.tatics.kickoff import OurKickoff



class CheckIfOurKickoff(LeafNode):
    def run(self, context: TickContext) -> RunResult:
        if context.is_team_color_yellow is None:
            return TaskStatus.RUNNING, None

        expected_cmd = (
            "PREPARE_KICKOFF_YELLOW"
            if context.is_team_color_yellow
            else "PREPARE_KICKOFF_BLUE"
        )

        if context.referee_command == expected_cmd:
            return TaskStatus.SUCCESS, None
        return TaskStatus.FAILURE, None


class OurKickoffAction(LeafNode):
    def run(self, context: TickContext) -> RunResult:
        if not context.ally_robots or context.on_positive_half is None:
            return TaskStatus.RUNNING, None

        executor = OurKickoff(
            ally_robots=context.ally_robots, on_positive_half=context.on_positive_half
        )

        return TaskStatus.SUCCESS, executor.execute()


class TheirKickoffAction(LeafNode):
    def run(self, context: TickContext) -> RunResult:
        if not context.ally_robots or context.on_positive_half is None:
            return TaskStatus.RUNNING, None

        # Carried over as-is from before this refactor: this branch builds OurKickoff,
        # not TheirKickoff. Left alone deliberately so the refactor does not change
        # what the robots do on the field.
        executor = OurKickoff(
            ally_robots=context.ally_robots, on_positive_half=context.on_positive_half
        )

        return TaskStatus.SUCCESS, executor.execute()


class Kickoff(Sequence):
    def __init__(self, name: str, deps: TreeDeps):
        """List with possible inputs to this state"""

        commands = ["PREPARE_KICKOFF_BLUE", "PREPARE_KICKOFF_YELLOW"]

        check_kickoff = CheckState("CheckKickoff", deps, commands)

        is_ours = CheckIfOurKickoff("CheckIfOurKickoff", deps)
        action_ours = OurKickoffAction("OurKickoffAction", deps)

        ours = Sequence("OurKickoff", deps, [is_ours, action_ours])

        action_theirs = TheirKickoffAction("TheirKickoffAction", deps)

        ours_or_theirs = Selector("OursOrTheirsKickoff", deps, [ours, action_theirs])

        super().__init__(name, deps, [check_kickoff, ours_or_theirs])
