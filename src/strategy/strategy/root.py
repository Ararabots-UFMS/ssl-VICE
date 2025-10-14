from strategy.behaviour import Selector


# from strategy.plays.freekick import FreeKick
# from strategy.coach.halt import Halt
from strategy.plays.kickoff import Kickoff
# from strategy.coach.penalty import Penalty
# from strategy.coach.stop import Stop
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

        self.add_children([Kickoff("Kickoff"), NormalStart("NormalStart")])

    def run(self):
        return super().run()
