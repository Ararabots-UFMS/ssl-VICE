from strategy.behaviour import Selector


# from strategy.coach.freekick import FreeKick
from strategy.plays.halt import Halt
# from strategy.coach.kickoff import Kickoff
# from strategy.coach.penalty import Penalty
# from strategy.coach.timeout import _Timeout
from strategy.plays.running import NormalStart
# from strategy.plays.defense import Defense

class RootTree(Selector):
    def __init__(self, name):
        super().__init__(name, [])
        #kickoff = Kickoff("Kickoff")
        #freekick = FreeKick("FreeKick")
        #stop = Stop("Stop")
        #penalty = Penalty("Penalty")
        #timeout = _Timeout("Timeout")
        #halt = Halt("Halt")
        #running = Running("Running")

        #self.add_children([stop, halt, kickoff, freekick, penalty, timeout, running])

        self.add_children([Halt("Halt")])

    def run(self):
        return super().run()
