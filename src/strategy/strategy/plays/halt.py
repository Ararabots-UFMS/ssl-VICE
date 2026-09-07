from strategy.behaviour import LeafNode, RunResult, Sequence
from strategy.commons.check_state import CheckState
from strategy.commons.task_status import TaskStatus
from strategy.context import TickContext, TreeDeps
from strategy.tatics.halt import HaltAction


class HaltActionNode(LeafNode):
    def run(self, context: TickContext) -> RunResult:
        if not context.ally_robots:
            return TaskStatus.RUNNING, None

        executor = HaltAction(context.ally_robots)

        return TaskStatus.SUCCESS, executor.execute()


class Halt(Sequence):
    def __init__(self, name: str, deps: TreeDeps):
        commands = ["HALT"]
        check_halt = CheckState("CheckHalt", deps, commands)
        action = HaltActionNode("HaltAction", deps)
        super().__init__(name, deps, [check_halt, action])
