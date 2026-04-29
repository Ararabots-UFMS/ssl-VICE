from dataclasses import dataclass

@dataclass
class ObstaclesManager:
    border_area: bool = False
    center_circle: bool = False

    @classmethod
    def from_service_response(cls, resp):
        return cls(
            border_area=bool(getattr(resp, 'border_area', False)),
            center_circle=bool(getattr(resp, 'center_circle', False))
        )

    def to_dict(self):
        return  {
            'border_area': self.border_area,
            'center_circle': self.center_circle
        }