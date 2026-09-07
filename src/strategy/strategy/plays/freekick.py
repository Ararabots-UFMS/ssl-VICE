from strategy.behaviour import LeafNode, RunResult, Selector, Sequence
from strategy.commons.check_state import CheckState
from strategy.commons.task_status import TaskStatus
from strategy.context import TickContext, TreeDeps
from strategy.tatics.freekick import OurFreekick, TheirFreekick

class CheckIfOurFreekick(LeafNode):
    def run(self, context: TickContext) -> RunResult:
        if context.is_team_color_yellow is None:
            return TaskStatus.RUNNING, None

        expected_cmd = (
            "DIRECT_FREE_YELLOW" if context.is_team_color_yellow else "DIRECT_FREE_BLUE"
        )

        if context.referee_command == expected_cmd:
            return TaskStatus.SUCCESS, None
        return TaskStatus.FAILURE, None


class OurFreekickAction(LeafNode):
    def run(self, context: TickContext) -> RunResult:
        if not context.has_robots_and_ball() or context.on_positive_half is None:
            return TaskStatus.RUNNING, None

        executor = OurFreekick(
            ally_robots=context.ally_robots,
            ball=context.ball,
            on_positive_half=context.on_positive_half,
        )

        return TaskStatus.SUCCESS, executor.execute()


class TheirFreekickAction(LeafNode):
    def run(self, context: TickContext) -> RunResult:
        if not context.has_robots_and_ball() or context.on_positive_half is None:
            return TaskStatus.RUNNING, None

        executor = TheirFreekick(
            ally_robots=context.ally_robots,
            ball=context.ball,
            on_positive_half=context.on_positive_half,
        )

        return TaskStatus.SUCCESS, executor.execute()


class Freekick(Sequence):
    def __init__(self, name: str, deps: TreeDeps):
        """List with possible inputs to this state"""

        commands = ["DIRECT_FREE_BLUE", "DIRECT_FREE_YELLOW"]

        check_freekick = CheckState("CheckFreekick", deps, commands)

        is_ours = CheckIfOurFreekick("CheckIfOurFreekick", deps)
        action_ours = OurFreekickAction("OurFreekickAction", deps)

        ours = Sequence("OurFreekick", deps, [is_ours, action_ours])

        action_theirs = TheirFreekickAction("TheirFreekickAction", deps)

        ours_or_theirs = Selector("OursOrTheirsFreeKick", deps, [ours, action_theirs])

        super().__init__(name, deps, [check_freekick, ours_or_theirs])
