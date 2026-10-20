from strategy.behaviour import LeafNode, RunResult, Sequence
from strategy.commons.check_state import CheckState
from strategy.commons.task_status import TaskStatus
from strategy.context import TickContext, TreeDeps
from strategy.tatics.running import JogoCorrido


class JogoCorridoAction(LeafNode):
    """Runs open play. Attack and defence are roles inside it, not tree branches."""

    def __init__(self, name: str, deps: TreeDeps):
        super().__init__(name, deps)
        # Built once: role hysteresis, aim lock and kick latches live in it.
        self.jogo = JogoCorrido()

    def run(self, context: TickContext) -> RunResult:
        if not context.has_robots_and_ball() or context.on_positive_half is None:
            return TaskStatus.RUNNING, None
        return TaskStatus.SUCCESS, self.jogo.comandos(context)


class NormalStart(Sequence):
    def __init__(self, name: str, deps: TreeDeps):
        can_i_start = CheckState("CheckState", deps, ["FORCE_START", "NORMAL_START"])
        jogo = JogoCorridoAction("JogoCorrido", deps)
        super().__init__(name, deps, [can_i_start, jogo])
