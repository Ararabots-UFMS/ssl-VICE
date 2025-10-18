from strategy.behaviour import Selector


# from strategy.coach.freekick import FreeKick  
# from strategy.coach.kickoff import Kickoff
# from strategy.coach.penalty import Penalty
from strategy.plays.freekick import Freekick
# from strategy.coach.halt import Halt
from strategy.plays.kickoff import Kickoff
# from strategy.coach.penalty import Penalty
from strategy.plays.stop import Stop
from strategy.plays.running import NormalStart

class RootTree(Selector):
    def __init__(self, name):
        super().__init__(name, [])

        #self.add_children([stop, halt, kickoff, freekick, penalty, timeout, running])

        self.add_children([Stop("Stop") , Kickoff("Kickoff"), Freekick("Freekick"), NormalStart("NormalStart")])


    def run(self):
        return super().run()
