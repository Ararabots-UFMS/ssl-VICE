from strategy.blackboard import Blackboard

from abc import abstractmethod
from enum import Enum

class TaskStatus(Enum):
    SUCCESS = 0
    FAILURE = 1
    RUNNING = 2
    
# Action can be a list of behaviour trees if the behaviour tree is in the coach
# and can be a movement profile and args for trajactory planning
    
class LeafNode():
    def __init__(self, name):
        self.name = name

    @abstractmethod
    def run(self):
        raise Exception("subclass must override run")
    
class TreeNode:
    pass

class TreeNode:
    def __init__(self, name, children):
        self.name = name
        self.children = []
        self.add_children(children)

    def add_children(self, children) -> None:
        for child in children:
            self.children.append(child)

    @abstractmethod
    def run(self):
        raise Exception("subclass must override run")


class Sequence(TreeNode):
    """
    A sequence runs each task in order until one fails,
    at which point it returns FAILURE. If all tasks succeed, a SUCCESS
    status is returned.  If a subtask is still RUNNING, then a RUNNING
    status is returned and processing continues until either SUCCESS
    or FAILURE is returned from the subtask.
    """

    def __init__(self, name, children):
        super().__init__(name, children)

    def run(self):
        print(self.name, end=" ")
        for c in self.children:
            status, action = c.run()
            if status != TaskStatus.SUCCESS:
                print(action)
                return status, action
        print(action)
        return status.SUCCESS, action


class Selector(TreeNode):
    """
    A selector runs each task in order until one succeeds,
    at which point it returns SUCCESS. If all tasks fail, a FAILURE
    status is returned.  If a subtask is still RUNNING, then a RUNNING
    status is returned and processing continues until either SUCCESS
    or FAILURE is returned from the subtask.
    """

    def __init__(self, name, children):
        super().__init__(name, children)

    def run(self):
        print(self.name, end=" ")
        for c in self.children:
            status, action = c.run()
            if status != TaskStatus.FAILURE:
                print(action)
                return status, action
        print("None")
        return TaskStatus.FAILURE, "None"
    
class BaseTree(Selector):
    def __init__(self, name, children):
        super().__init__(name, children)

    def run(self):
        return super().run()