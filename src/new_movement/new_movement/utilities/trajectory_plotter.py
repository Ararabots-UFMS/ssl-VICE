import matplotlib.pyplot as plt

class TrajectoryPlotter:
    def __init__(self):
        pass

    def plot(self, trajectory, show=True, label="Trajectory", color="blue"):
        positions = trajectory.to_list(time_step = 0.01)
        xs = []
        ys = []
        for position in positions:
            xs.append(position.x)
            ys.append(position.y)

        plt.plot(xs, ys, marker='o', label=label, color=color)
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.title("Trajectory Plot")
        plt.legend()
        plt.grid(True)
        if show:
            plt.show()