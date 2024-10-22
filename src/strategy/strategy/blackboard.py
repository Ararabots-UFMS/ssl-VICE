from system_interfaces.msg import GUIMessage, GameData, VisionGeometry
from threading import Lock


class SingletonMeta(type):
    """
    This is a thread-safe implementation of Singleton.
    """

    _instances = {}

    _lock: Lock = Lock()
    """
    We now have a lock object that will be used to synchronize threads during
    first access to the Singleton.
    """

    def __call__(cls, *args, **kwargs):
        """
        Possible changes to the value of the `__init__` argument do not affect
        the returned instance.
        """
        # Now, imagine that the program has just been launched. Since there's no
        # Singleton instance yet, multiple threads can simultaneously pass the
        # previous conditional and reach this point almost at the same time. The
        # first of them will acquire lock and will proceed further, while the
        # rest will wait here.
        with cls._lock:
            # The first thread to acquire the lock, reaches this conditional,
            # goes inside and creates the Singleton instance. Once it leaves the
            # lock block, a thread that might have been waiting for the lock
            # release may then enter this section. But since the Singleton field
            # is already initialized, the thread won't create a new object.
            if cls not in cls._instances:
                instance = super().__call__(*args, **kwargs)
                cls._instances[cls] = instance
        return cls._instances[cls]


""" Made some changes to run just the referee subscriber """


class Blackboard(metaclass=SingletonMeta):
    def __init__(self) -> None:
        self.ally_robots = {}
        self.enemy_robots = {}
        self.balls = {}
        self.gui = GUIMessage()
        self.referee = GameData()
        self.geometry = VisionGeometry()

        # TODO: Remove this line
        self.gui.is_team_color_yellow = True

    def update_from_vision_message(self, message):
        if message is None:
            return
        elif self.gui.is_team_color_yellow:
            self.ally_robots = {ally.id: ally for ally in message.yellow_robots}
            self.enemy_robots = {enemy.id: enemy for enemy in message.blue_robots}
        else:
            self.ally_robots = {ally.id: ally for ally in message.blue_robots}
            self.enemy_robots = {enemy.id: enemy for enemy in message.yellow_robots}

        self.balls = message.balls

    def update_from_gamecontroller_message(self, message: GameData):
        self.referee = message

    def update_from_gui_message(self, message: GUIMessage):
        self.gui = message

    def update_from_geometry(self, message: VisionGeometry):
        self.geometry = message