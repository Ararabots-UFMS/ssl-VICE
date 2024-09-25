class Robot(Node):
    def __init__(self, id, position, velocity, behaviour_tree) -> None:
        super().__init__('robot')
        self.id = id
        self.position = position
        self.velocity = velocity
        self.behaviour_tree = behaviour_tree
        self.address = None
        self.pid = None
        
        self.timer = self.create_timer(0.1, self.run)
    
    def run(self):
        self.behaviour_tree.run()