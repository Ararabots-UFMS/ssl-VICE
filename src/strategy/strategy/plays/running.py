from strategy.behaviour import Selector

from strategy.tatics.triangle import TriangleFormation

class NormalStart(Selector):
    def __init__(self, name, node):
        super().__init__(name, [])

        triangle_formation = TriangleFormation("TriangleFormation")
        self.add_children([triangle_formation])

    def run(self):
        return super().run()
