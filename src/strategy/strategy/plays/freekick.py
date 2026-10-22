from strategy.behaviour import LeafNode, RunResult, Selector, Sequence
from strategy.commons.check_state import CheckState
from strategy.commons.task_status import TaskStatus
from strategy.context import TickContext, TreeDeps
from strategy.tatics.freekick import (
    EstadoFreekick,
    MedidasCampo,
    OurFreekick,
    TheirFreekick,
)


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


class _FreekickAction(LeafNode):
    """Shared by both sides. The play's memory is injected so it survives ticks."""

    tatica = None

    def __init__(self, name: str, deps: TreeDeps, estado: EstadoFreekick):
        super().__init__(name, deps)
        self.estado = estado

    def run(self, context: TickContext) -> RunResult:
        if not context.has_robots_and_ball() or context.on_positive_half is None:
            return TaskStatus.RUNNING, None

        # Measurements come from vision when it reports them, Division B otherwise.
        medidas = (MedidasCampo.da_geometria(context.geometry)
                   if context.geometry is not None else MedidasCampo())
        executor = self.tatica(
            ally_robots=context.ally_robots,
            ball=context.ball,
            on_positive_half=context.on_positive_half,
            enemy_robots=context.enemy_robots,
            estado=self.estado,
            medidas=medidas,
        )
        return executor.execute()


class OurFreekickAction(_FreekickAction):
    tatica = OurFreekick


class TheirFreekickAction(_FreekickAction):
    tatica = TheirFreekick


class Freekick(Sequence):
    """Free-kick play. Owns the memory both actions share (kicker, timer)."""

    def __init__(self, name: str, deps: TreeDeps):
        self.estado = EstadoFreekick()
        self._ultimo_comando = None

        commands = ["DIRECT_FREE_BLUE", "DIRECT_FREE_YELLOW"]

        check_freekick = CheckState("CheckFreekick", deps, commands)

        is_ours = CheckIfOurFreekick("CheckIfOurFreekick", deps)
        action_ours = OurFreekickAction("OurFreekickAction", deps, self.estado)

        ours = Sequence("OurFreekick", deps, [is_ours, action_ours])

        action_theirs = TheirFreekickAction("TheirFreekickAction", deps, self.estado)

        ours_or_theirs = Selector("OursOrTheirsFreeKick", deps, [ours, action_theirs])

        super().__init__(name, deps, [check_freekick, ours_or_theirs])

    def run(self, context: TickContext) -> RunResult:
        # Each free kick is a new play: without this the 10 s limit and the
        # double-touch guard of one kick leak into the next.
        if context.referee_command != self._ultimo_comando:
            self.estado.reiniciar()
            self._ultimo_comando = context.referee_command
        return super().run(context)
