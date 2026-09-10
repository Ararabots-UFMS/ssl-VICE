"""Control sequence merging."""


class ControlMerger:
    """Merges scalar axis controls at their shared time boundaries."""

    def __init__(self, time_epsilon=1.0e-7):
        self.time_epsilon = time_epsilon

    def duration(self, control):
        return sum(segment[1] for segment in control)

    def vector_with_scalar(self, vector_control, scalar_control):
        if not vector_control or not scalar_control:
            return []
        result = []
        first_index = second_index = 0
        first_elapsed = second_elapsed = 0.0
        while first_index < len(vector_control) and second_index < len(scalar_control):
            first = vector_control[first_index]
            second = scalar_control[second_index]
            first_end = first_elapsed + first[1]
            second_end = second_elapsed + second[1]
            end = min(first_end, second_end)
            result.append([first[0] + [second[0]], end - max(first_elapsed, second_elapsed)])
            if abs(end - first_end) <= self.time_epsilon:
                first_index += 1
                first_elapsed = first_end
            if abs(end - second_end) <= self.time_epsilon:
                second_index += 1
                second_elapsed = second_end
        return result

    def scalar_axes(self, controls):
        if not controls:
            return []
        merged = [[[segment[0]], segment[1]] for segment in controls[0]]
        for control in controls[1:]:
            merged = self.vector_with_scalar(merged, control)
        return merged
