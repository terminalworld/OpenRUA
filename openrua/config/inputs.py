"""Content identities for declared build inputs, independent of host locations."""

import hashlib
import json
from pathlib import Path

from openrua import __version__


def file_digest(path: Path) -> str:
    digest = hashlib.sha256()
    files = [path] if path.is_file() else sorted(path.rglob('*'))
    if not path.exists():
        raise ValueError(f'build input is missing: {path}; restore the declared project files')
    for file in files:
        if '.git' in file.relative_to(path.parent if path.is_file() else path).parts:
            continue
        if file.is_file():
            digest.update(str(file.relative_to(path.parent if path.is_file() else path)).encode())
            digest.update(str(file.stat().st_mode & 0o777).encode())
            digest.update(file.read_bytes())
    return digest.hexdigest()


def install_digest(install: dict) -> str:
    def content(value, key=''):
        if isinstance(value, dict):
            return {name: content(item, name) for name, item in value.items()
                    if name != 'here' or '{here}' in value.get('shell', '')}
        if isinstance(value, list):
            return [content(item) for item in value]
        if key in {'requirements', 'patch', 'here'} and value:
            return file_digest(Path(value))
        return value
    return hashlib.sha256(json.dumps([__version__, content(install)], sort_keys=True).encode()).hexdigest()


def driver_images(backend: dict) -> None:
    """Resolve build names and explicit launch slots from their source contents."""
    builds = backend.get('build') or {}
    images = {}
    for name, directory in builds.items():
        base = name.rsplit(':', 1)[0] if ':' in name.rsplit('/', 1)[-1] else name
        images[name] = f'{base}:{file_digest(Path(directory))[:16]}'
    backend['images'] = images
    backend['build'] = {images[name]: directory for name, directory in builds.items()}
    if backend.get('image') in images:
        backend['image'] = images[backend['image']]
    for holder, key in [(backend, 'launch'), *((check, 'run') for check in backend.get('startup_checks', []))]:
        if holder.get(key):
            for name, image in images.items():
                holder[key] = holder[key].replace('{images[' + name + ']}', image)
            if '{images[' in holder[key]:
                raise ValueError('launch names an unknown build image; use a name from backend.build')
