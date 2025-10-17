from strategy.behaviour import Selector


# from strategy.plays.freekick import FreeKick
# from strategy.coach.halt import Halt
from strategy.plays.kickoff import Kickoff
# from strategy.coach.penalty import Penalty
from strategy.plays.stop import Stop
from strategy.plays.running import NormalStart

class RootTree(Selector):
    def __init__(self, name):
        super().__init__(name, [])

        self.add_children([Stop("Stop"), Kickoff("Kickoff"), NormalStart("NormalStart")])

    def run(self):
        return super().run()
