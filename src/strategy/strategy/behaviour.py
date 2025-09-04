from abc import abstractmethod
from enum import Enum
from typing import List, Optional


class TaskStatus(Enum):
    SUCCESS = 0
    FAILURE = 1
    RUNNING = 2


class LeafNode:
    def __init__(self, name: str):
        self.name = name

    @abstractmethod
    def run(self) -> TaskStatus:
        raise NotImplementedError


class TreeNode:
    def __init__(self, name: str, children: Optional[List[LeafNode]] = None):
        self.name = name
        self.children = []
        if children:
            self.add_children(children)

    def add_children(self, children) -> None:
        for child in children:
            self.children.append(child)

    @abstractmethod
    def run(self) -> TaskStatus:
        raise NotImplementedError


class Sequence(TreeNode):
    def __init__(self, name: str, children: Optional[List[LeafNode]] = None):
        super().__init__(name, children)

    def run(self) -> TaskStatus:
        if not self.children:
            return TaskStatus.SUCCESS
        for c in self.children:
            status = c.run()
            if status != TaskStatus.SUCCESS:
                return status
        return TaskStatus.SUCCESS


class Selector(TreeNode):
    def __init__(self, name: str, children: Optional[List[LeafNode]] = None):
        super().__init__(name, children)

    def run(self) -> TaskStatus:
        if not self.children:
            return TaskStatus.FAILURE
        for c in self.children:
            status = c.run()
            if status != TaskStatus.FAILURE:
                return status
        return TaskStatus.FAILURE


class BaseTree(Selector):
    def __init__(self, name: str, children: Optional[List[LeafNode]] = None):
        super().__init__(name, children)

    def run(self) -> TaskStatus:
        return super().run()
