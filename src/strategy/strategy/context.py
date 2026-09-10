"""What the behaviour tree is allowed to know, and how it gets there.

Two separate things live here because they answer two different questions.

``TickContext`` is the world as it looked at the top of one tick: an immutable
snapshot built once by the strategy node and passed down through ``run()``. Nothing
inside the tree can change it, and no subscription callback can change it mid-tick,
so every node in one traversal sees the same field. That is what removes the tearing
the old singleton blackboard had, where a vision callback could land between two
reads and leave a node comparing a ball from one frame against robots from the next.

``TreeDeps`` is everything that outlives a tick. It is handed to each node at
construction, so what a node depends on is visible in its signature instead of being
reached for through a global.

Neither of these imports rclpy. A behaviour tree node can be built and ticked in a
plain unit test with a hand-written context and no ROS graph at all.
"""

from __future__ import annotations

from dataclasses import dataclass
from types import MappingProxyType
from typing import Any, Mapping, Optional, Tuple

# Shared empty mapping. Safe as a dataclass default precisely because it cannot be
# mutated, which is the same reason the real fields use a mapping proxy.
EMPTY_MAP: Mapping[int, Any] = MappingProxyType({})


class NullLogger:
    """A logger that discards everything, for tests and for headless tree building."""

    def debug(self, *_args, **_kwargs) -> None: ...

    def info(self, *_args, **_kwargs) -> None: ...

    def warn(self, *_args, **_kwargs) -> None: ...

    def warning(self, *_args, **_kwargs) -> None: ...

    def error(self, *_args, **_kwargs) -> None: ...


@dataclass(frozen=True)
class TreeDeps:
    """Collaborators that outlive a single tick, injected at construction.

    Today that is just a logger, because hoisting the GetGameConfig calls into the
    strategy node left the tree with no other long-lived needs. This is the seam
    where the next one goes: a command sink, a service client, per-play memory. Add
    it here and it stays visible in every constructor rather than becoming a global.
    """

    logger: Any

    @classmethod
    def null(cls) -> "TreeDeps":
        """Dependencies that do nothing. Intended for tests."""
        return cls(logger=NullLogger())


@dataclass(frozen=True)
class GameConfig:
    """The GetGameConfig reply, frozen.

    Fetched once by the strategy node and reused every tick, replacing the six
    separate leaves that each polled the service on their own timer. It is ``None``
    on the context until the service answers, and the nodes that need it report
    RUNNING until then, exactly as they did before.
    """

    is_team_color_yellow: bool = False
    on_positive_half: bool = False
    robot_count: int = 0

    @classmethod
    def from_response(cls, response: Any) -> "GameConfig":
        return cls(
            is_team_color_yellow=bool(response.is_team_color_yellow),
            on_positive_half=bool(response.on_positive_half),
            robot_count=int(response.robot_count),
        )


@dataclass(frozen=True)
class TickContext:
    """One tick's view of the world. Built once per tick, read many times.

    The containers are genuinely immutable: a mapping proxy for the robots, a tuple
    for the balls. The ROS message objects inside them are not, so the rule the tree
    has to keep is simply that nobody writes to them. Nothing in the tree does, and
    the subscription callback that receives them never touches a message after
    handing it over.
    """

    ally_robots: Mapping[int, Any] = EMPTY_MAP
    enemy_robots: Mapping[int, Any] = EMPTY_MAP
    balls: Tuple[Any, ...] = ()
    referee_command: str = ""
    config: Optional[GameConfig] = None
    vision_capture_stamp: float = 0.0

    @classmethod
    def from_game_state(
        cls, msg: Any, config: Optional[GameConfig] = None
    ) -> "TickContext":
        """Freeze one GameState message into a context.

        Every projection the tree used to build for itself, nineteen times over,
        happens once here. Robots come out keyed by id because that is how every
        tactic wants them.
        """
        return cls(
            ally_robots=MappingProxyType({r.id: r for r in msg.ally_robots}),
            enemy_robots=MappingProxyType({r.id: r for r in msg.enemy_robots}),
            balls=tuple(msg.balls),
            referee_command=msg.referee.command,
            config=config,
            vision_capture_stamp=float(getattr(msg, "vision_capture_stamp", 0.0)),
        )

    @property
    def ball(self) -> Optional[Any]:
        """The tracked ball, or None when vision has not reported one."""
        return self.balls[0] if self.balls else None

    @property
    def on_positive_half(self) -> Optional[bool]:
        """None until the game config arrives, then the configured half."""
        return None if self.config is None else self.config.on_positive_half

    @property
    def is_team_color_yellow(self) -> Optional[bool]:
        """None until the game config arrives, then the configured team color."""
        return None if self.config is None else self.config.is_team_color_yellow

    def has_robots_and_ball(self) -> bool:
        """True when there is enough of a world to plan against."""
        return bool(self.ally_robots) and bool(self.balls)
