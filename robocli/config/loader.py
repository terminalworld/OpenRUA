"""Reading and layering config files: yaml in, validated dicts out.

Every file is validated against its schema model; unknown keys are
errors. Consumers read plain dicts (``dump()``): defaults filled in,
absent optionals left out.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml
from pydantic import BaseModel, ValidationError

from robocli.config.schema import UserConfig
from robocli.errors import ConfigError


def validate(model: type[BaseModel], data: Any, source: str | Path) -> BaseModel:
    """Validate ``data`` (parsed yaml) against ``model``; raise ConfigError
    naming ``source`` and each bad key path."""
    try:
        return model.model_validate(data if data is not None else {})
    except ValidationError as e:
        lines = []
        for err in e.errors():
            loc = ".".join(str(x) for x in err["loc"]) or "<root>"
            lines.append(f"  {loc}: {err['msg']}")
        raise ConfigError(f"{source}: does not fit the {model.__name__} schema\n"
                          + "\n".join(lines),
                          hint="robocli config schema prints every key and its meaning") from None


def dump(model: BaseModel) -> dict:
    """The dict consumers read: defaults filled in, absent optionals
    (None) left out so ``cfg.get(key, default)`` keeps its meaning."""
    return model.model_dump(exclude_none=True, by_alias=True)


def load_yaml(path: Path) -> Any:
    try:
        return yaml.safe_load(path.read_text())
    except yaml.YAMLError as e:
        raise ConfigError(f"{path}: not valid YAML: {e}") from None


def load_user_config(path: Path) -> UserConfig:
    """The user's defaults file; absent = no defaults."""
    if not path.is_file():
        return UserConfig()
    return validate(UserConfig, load_yaml(path), path)


def layer_agent(bench_agent: dict, *defaults: UserConfig) -> dict:
    """The benchmark's agent section over the defaults files, lowest
    layer first (package defaults, then the user's file). A defaults
    file contributes only the keys it wrote; ``options`` merge key by
    key, every other key is replaced by the higher layer."""
    out: dict = {}
    layers = [d.agent.model_dump(exclude_unset=True, exclude_none=True) for d in defaults]
    for layer in layers + [bench_agent]:
        for k, v in layer.items():
            if k == "options":
                out["options"] = {**out.get("options", {}), **(v or {})}
            else:
                out[k] = v
    return out
