class Robot():
    def __init__(self, id, position, velocity, behaviour_tree) -> None:
        self.id = id
        self.position = position
        self.velocity = velocity
        self.behaviour_tree = behaviour_tree
    
    def run(self):
        self.behaviour_tree.run()