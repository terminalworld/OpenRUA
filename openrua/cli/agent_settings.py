"""Resolve one agent's user defaults consistently, including explicit pins."""
from pathlib import Path

from openrua import agents, config
from openrua.config import paths


def resolve(home: Path, spec: str | None):
    package = config.load_user_config(paths.package_config_path())
    user = config.load_user_config(paths.config_path(home))
    requested = spec or config.default_agent_name(package, user) or ''
    name, pin = agents.split_pin(requested)
    if paths.is_path(name):
        name = str(Path(name).expanduser().resolve())
    selected = config.layer_agent({}, package, user, agent=name or None)
    if pin:
        selected['version'] = pin
    return agents.get(selected['name'], home, version=selected.get('version')), selected
