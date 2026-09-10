from strategy.behaviour import LeafNode, RunResult, Selector, Sequence
from strategy.commons.check_state import CheckState
from strategy.commons.task_status import TaskStatus
from strategy.context import TickContext, TreeDeps
from strategy.tatics.stop import goAwayFromBall

# Minimum legal distance from the ball during a STOP, in millimetres.
STOP_KEEPOUT_MM = 500.0


class CheckDistance(LeafNode):
    """Succeeds when every ally is already outside the STOP keep-out circle."""

    def _all_clear_of_ball(self, context: TickContext) -> bool:
        ball = context.ball
        for robot in context.ally_robots.values():
            distance = (
                (robot.position_x - ball.position_x) ** 2
                + (robot.position_y - ball.position_y) ** 2
            ) ** 0.5
            if distance < STOP_KEEPOUT_MM:
                return False

        return True

    def run(self, context: TickContext) -> RunResult:
        if not context.has_robots_and_ball():
            return TaskStatus.RUNNING, None

        if self._all_clear_of_ball(context):
            return TaskStatus.SUCCESS, None
        return TaskStatus.FAILURE, None


class KeepPosition(LeafNode):
    def run(self, context: TickContext) -> RunResult:
        return TaskStatus.SUCCESS, None


class GetoffBall(LeafNode):
    def run(self, context: TickContext) -> RunResult:
        # on_positive_half is checked as well as the world state: goAwayFromBall
        # branches on it, and a None there reads as the negative half, which would
        # send everyone to the wrong side of the field.
        if not context.has_robots_and_ball() or context.on_positive_half is None:
            return TaskStatus.RUNNING, None

        go_away = goAwayFromBall(context.ally_robots, context.ball, context.on_positive_half)

        return TaskStatus.SUCCESS, go_away.execute()


class Stop(Sequence):
    def __init__(self, name: str, deps: TreeDeps):
        game_state_is_stop = CheckState("game_state_is_stop", deps, ["STOP"])

        robot_is_far_from_ball = CheckDistance("robot_is_far_from_ball", deps)

        maintain_current_position = KeepPosition("maintain_current_position", deps)

        move_away_from_ball = GetoffBall("move_away_from_ball", deps)

        allowed_to_stop = Sequence(
            "allowed_to_stop",
            deps,
            [robot_is_far_from_ball, maintain_current_position],
        )

        decide_stop_or_move_away = Selector(
            "decide_stop_or_move_away",
            deps,
            [allowed_to_stop, move_away_from_ball],
        )

        super().__init__(name, deps, [game_state_is_stop, decide_stop_or_move_away])
