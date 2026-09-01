"""Colour names to RGBA.

`VehicleState.color` is a name string to keep it beginner-friendly. An unknown
name deliberately renders magenta rather than a sensible default, so a typo is
obvious on screen instead of silently looking fine.
"""

from std_msgs.msg import ColorRGBA

UNKNOWN_COLOR = (1.0, 0.0, 1.0)

_NAMED: dict[str, tuple[float, float, float]] = {
    'red': (0.87, 0.16, 0.16),
    'blue': (0.15, 0.39, 0.92),
    'green': (0.13, 0.69, 0.30),
    'yellow': (0.98, 0.83, 0.15),
    'orange': (0.96, 0.55, 0.11),
    'purple': (0.58, 0.24, 0.83),
    'pink': (0.95, 0.45, 0.72),
    'cyan': (0.15, 0.78, 0.85),
    'white': (0.96, 0.96, 0.96),
    'black': (0.12, 0.12, 0.14),
    'grey': (0.55, 0.55, 0.58),
    'gray': (0.55, 0.55, 0.58),
    'brown': (0.48, 0.33, 0.20),
    'silver': (0.75, 0.76, 0.78),
}

# Scenery, so the whole palette lives in one file.
GRASS = (0.42, 0.65, 0.34)
TREE = (0.24, 0.45, 0.22)
ASPHALT = (0.29, 0.30, 0.32)
ROAD_MARKING = (0.95, 0.95, 0.92)
RAIL = (0.38, 0.34, 0.30)
SLEEPER = (0.34, 0.26, 0.19)
BUILDING = (0.78, 0.72, 0.63)
ROOF = (0.55, 0.31, 0.26)
PLATFORM = (0.70, 0.69, 0.66)
BARRIER = (0.90, 0.24, 0.20)
LABEL = (0.98, 0.98, 0.95)

# Traffic light lamps, lit and unlit.
LAMP_RED = (0.90, 0.16, 0.16)
LAMP_YELLOW = (0.98, 0.78, 0.12)
LAMP_GREEN = (0.20, 0.80, 0.32)
LAMP_OFF = (0.16, 0.16, 0.18)
LAMP_HOUSING = (0.22, 0.22, 0.24)


def rgba(color: tuple[float, float, float], alpha: float = 1.0) -> ColorRGBA:
    """Build a ColorRGBA from an (r, g, b) triple."""
    return ColorRGBA(r=color[0], g=color[1], b=color[2], a=alpha)


def by_name(name: str, alpha: float = 1.0) -> ColorRGBA:
    """Look up a colour by name, falling back to a loud magenta."""
    return rgba(_NAMED.get(name.strip().lower(), UNKNOWN_COLOR), alpha)


def is_known(name: str) -> bool:
    """Return whether a colour name is one the sim recognises."""
    return name.strip().lower() in _NAMED


def known_names() -> list[str]:
    """Return the colour names recruits can use, for docs and error messages."""
    return sorted(_NAMED)
