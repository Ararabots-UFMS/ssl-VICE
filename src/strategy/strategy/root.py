from strategy.behaviour import Selector
from strategy.context import TreeDeps
from strategy.plays.freekick import Freekick
from strategy.plays.halt import Halt
from strategy.plays.kickoff import Kickoff
from strategy.plays.running import NormalStart
from strategy.plays.stop import Stop


class RootTree(Selector):
    def __init__(self, name: str, deps: TreeDeps):
        super().__init__(
            name,
            deps,
            [
                Stop("Stop", deps),
                Halt("Halt", deps),
                Kickoff("Kickoff", deps),
                Freekick("Freekick", deps),
                NormalStart("NormalStart", deps),
            ],
        )
