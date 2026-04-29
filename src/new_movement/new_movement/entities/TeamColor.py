from dataclasses import dataclass

@dataclass
class TeamColor:
    is_yellow: bool = False

    @classmethod
    def from_service_response(cls, resp):
        return cls(is_yellow=bool(getattr(resp, 'is_team_color_yellow', False)))

    def __bool__(self):
        return self.is_yellow