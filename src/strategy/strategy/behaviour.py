"""Behaviour tree primitives.

These are plain Python objects. They do not inherit from rclpy's Node, do not
subscribe to anything, and cannot reach the ROS graph. A node receives what it needs
in two ways: long-lived collaborators through ``deps`` at construction, and the
world through the ``TickContext`` passed into ``run()``.

That makes a traversal a pure function of its context, so a test builds a context,
calls ``run`` and asserts on the returned skill, with no ROS running.
"""

from abc import ABC, abstractmethod
from typing import Any, Iterable, List, Optional, Sequence as SequenceT, Tuple

from strategy.context import TickContext, TreeDeps
from strategy.commons.task_status import TaskStatus


# What every run() returns: how the node finished, and the skill it wants executed.
RunResult = Tuple[TaskStatus, Optional[Any]]

class BehaviourNode(ABC):
    """Common base for leaves and composites."""

    def __init__(self, name: str, deps: TreeDeps):
        self.name = name
        self.deps = deps

    @property
    def logger(self):
        return self.deps.logger

    @abstractmethod
    def run(self, context: TickContext) -> RunResult:
        raise NotImplementedError("subclass must override run")


class LeafNode(BehaviourNode):
    """A node with no children. Reads the context, returns a status and a skill."""

    children: SequenceT["BehaviourNode"] = ()


class TreeNode(BehaviourNode):
    """A node that delegates to children."""

    def __init__(self, name: str, deps: TreeDeps, children: Iterable[BehaviourNode]):
        super().__init__(name, deps)
        self.children: List[BehaviourNode] = []
        self.add_children(children)

    def add_children(self, children: Iterable[BehaviourNode]) -> None:
        for child in children:
            self.children.append(child)


class Sequence(TreeNode):
    """
    A sequence runs each task in order until one fails,
    at which point it returns FAILURE. If all tasks succeed, a SUCCESS
    status is returned.  If a subtask is still RUNNING, then a RUNNING
    status is returned and processing continues until either SUCCESS
    or FAILURE is returned from the subtask.
    """

    def run(self, context: TickContext) -> RunResult:
        # Seeded so an empty sequence returns cleanly instead of raising on an
        # unbound name, which is what the previous version did.
        action = None
        for c in self.children:
            status, action = c.run(context)
            if status != TaskStatus.SUCCESS:
                return status, action
        return TaskStatus.SUCCESS, action


class Selector(TreeNode):
    """
    A selector runs each task in order until one succeeds,
    at which point it returns SUCCESS. If all tasks fail, a FAILURE
    status is returned.  If a subtask is still RUNNING, then a RUNNING
    status is returned and processing continues until either SUCCESS
    or FAILURE is returned from the subtask.
    """

    def run(self, context: TickContext) -> RunResult:
        for c in self.children:
            status, action = c.run(context)
            if status != TaskStatus.FAILURE:
                return status, action
        return TaskStatus.FAILURE, None
