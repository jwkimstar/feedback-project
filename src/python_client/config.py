from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class NetworkConfig:
    multicast_group: str = "239.255.1.1"
    multicast_port: int = 49707
    command_port: int | None = None
    socket_timeout_seconds: float = 2.0
    beacon_wait_seconds: float = 5.0
    rpos_hz: int = 10


@dataclass(frozen=True)
class PathsConfig:
    artifacts_dir: Path = Path("artifacts")


DEFAULT_NETWORK_CONFIG = NetworkConfig()
DEFAULT_PATHS_CONFIG = PathsConfig()
