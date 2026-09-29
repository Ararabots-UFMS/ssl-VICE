import os
import time

from rclpy.node import Node
from abc import abstractmethod
from enum import Enum

class TaskStatus(Enum):
    SUCCESS = 0
    FAILURE = 1
    RUNNING = 2

class LeafNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.name = name

    @abstractmethod
    def run(self):
        raise Exception("subclass must override run")

class TreeNode(Node):
    def __init__(self, name, children):
        super().__init__(name)
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
        for c in self.children:
            status, action = c.run()
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

    def __init__(self, name, children):
        super().__init__(name, children)

    def run(self):
        for c in self.children:
            status, action = c.run()
            if status != TaskStatus.FAILURE:
                # T-5: UM RUNNING AQUI BLOQUEIA TODOS OS IRMAOS SEGUINTES.
                #
                # Esta e a semantica classica do Selector e a bola parada depende
                # dela - nao foi alterada. O que muda e que ela deixa de ser
                # SILENCIOSA: ja congelou o time inteiro duas vezes (deslocamento
                # de 2 mm em 24,5 s), e nas duas o log nao tinha uma linha sequer
                # dizendo quem estava segurando a arvore.
                #
                # Sai so com DIAG_JOGO, e so quando o filho devolve RUNNING - em
                # operacao normal nao imprime nada.
                if status == TaskStatus.RUNNING and os.environ.get("DIAG_JOGO"):
                    _agora = time.monotonic()
                    if _agora - getattr(self, "_ultimo_aviso_running", 0.0) > 1.0:
                        self._ultimo_aviso_running = _agora
                        _irmaos = [o.name for o in self.children
                                   if o is not c and hasattr(o, "name")]
                        print("[BT] %s parou em RUNNING no filho '%s'; nao rodaram: %s"
                              % (self.name, getattr(c, "name", "?"), _irmaos),
                              flush=True)
                return status, action
        return TaskStatus.FAILURE, None

class BaseTree(Selector):
    def __init__(self, name, children):
        super().__init__(name, children)

    def run(self):
        return super().run()
