from utils.math_util import Vector2D

from strategy.behaviour import LeafNode, RunResult, Selector, Sequence
from strategy.commons.check_state import CheckState
from strategy.commons.task_status import TaskStatus
from strategy.context import TickContext, TreeDeps
from strategy.tatics.running import Atack

# Radius of the circle around the field centre that decides attack versus defence.
ATTACK_BORDER_RADIUS_MM = 500.0


class CheckAtack(LeafNode):
    """Succeeds when the ball has crossed into the half we attack towards."""

    def _get_border_circle(self, on_positive_half: bool) -> Vector2D:
        if on_positive_half:
            return Vector2D(ATTACK_BORDER_RADIUS_MM, 0.0)
        return Vector2D(-ATTACK_BORDER_RADIUS_MM, 0.0)

    def run(self, context: TickContext) -> RunResult:
        if context.ball is None or context.on_positive_half is None:
            return TaskStatus.RUNNING, None

        border_circle = self._get_border_circle(context.on_positive_half)

        if context.on_positive_half:
            if context.ball.position_x < border_circle.x:
                return TaskStatus.SUCCESS, None
        else:
            if context.ball.position_x > border_circle.x:
                return TaskStatus.SUCCESS, None

        return TaskStatus.FAILURE, None


class AtackAction(LeafNode):
    def run(self, context: TickContext) -> RunResult:
        if not context.has_robots_and_ball() or context.on_positive_half is None:
            return TaskStatus.RUNNING, None

        atacker = Atack(
            ally_robots=context.ally_robots,
            enemy_robots=context.enemy_robots,
            ball=context.ball,
            on_positive_half=context.on_positive_half,
        )

        return TaskStatus.SUCCESS, atacker.execute()


class DefenseAction(LeafNode):
    def run(self, context: TickContext) -> RunResult:
        if not context.has_robots_and_ball() or context.on_positive_half is None:
            return TaskStatus.RUNNING, None

        # Carried over as-is from before this refactor: this branch builds Atack, not
        # Defense. Left alone deliberately so the refactor does not change what the
        # robots do on the field.
        atacker = Atack(
            ally_robots=context.ally_robots,
            enemy_robots=context.enemy_robots,
            ball=context.ball,
            on_positive_half=context.on_positive_half,
        )

        return TaskStatus.SUCCESS, atacker.execute()


class NormalStart(Sequence):
    def __init__(self, name: str, deps: TreeDeps):
        commands = ["FORCE_START", "NORMAL_START"]

        can_i_start = CheckState("CheckState", deps, commands)

        i_can_atack = CheckAtack("CheckAtack", deps)

        atack_action = AtackAction("AtackAction", deps)

        defense_action = DefenseAction("DefenseAction", deps)

        can_i_atack = Sequence("CanIAttack", deps, [i_can_atack, atack_action])

        start = Selector("Start", deps, [can_i_atack, defense_action])

        super().__init__(name, deps, [can_i_start, start])
