"""Resolve one agent's user defaults consistently, including explicit pins."""
from pathlib import Path

from openrua import agents, config
from openrua.config import paths


def project_model(saved: dict, requested: str | None) -> str | None:
    """Keep a project's model only when it belongs to the chosen agent."""
    if requested is None:
        return saved.get('model')
    return saved.get('model') if paths.agent_key(requested) == paths.agent_key(saved.get('agent') or '') else None


def resolve(home: Path, spec: str | None):
    package = config.load_user_config(paths.package_config_path())
    user = config.load_user_config(paths.config_path(home))
    requested = spec or config.default_agent_name(package, user) or ''
    selected = config.layer_agent({}, package, user, agent=requested or None)
    _, pin = paths.split_pin(selected['name'])
    selected['name'] = paths.agent_key(selected['name'])
    if pin:
        selected['version'] = pin
    return agents.get(selected['name'], home, version=selected.get('version')), selected
