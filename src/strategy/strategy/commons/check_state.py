from typing import Sequence

from strategy.behaviour import LeafNode, RunResult
from strategy.context import TickContext, TreeDeps
from strategy.commons.task_status import TaskStatus


class CheckState(LeafNode):
    """Succeeds when the referee command is one of the states this play answers to."""

    def __init__(self, name: str, deps: TreeDeps, desired_states: Sequence[str]):
        super().__init__(name, deps)
        self.desired_states = desired_states

    def run(self, context: TickContext) -> RunResult:
        return (
            (TaskStatus.SUCCESS, None)
            if context.referee_command in self.desired_states
            else (TaskStatus.FAILURE, None)
        )
