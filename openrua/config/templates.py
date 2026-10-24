"""Copy selected bundled declarations into a project's maintained files."""

from pathlib import Path
import re
import shutil

import yaml

from openrua.config import paths


KINDS = {'robot': 'robots', 'sim': 'simulators', 'bench': 'benchmarks'}


def instantiate(project: Path, choices: dict) -> dict:
    """Create missing template files and return their explicit project paths."""
    source_root = paths.package_config_path().parent.resolve()
    target_root = project.resolve() / 'configs'
    visited = set()

    def copy_file(source):
        target = target_root / source.relative_to(source_root)
        if target.is_symlink() or not target.resolve().is_relative_to(target_root):
            raise ValueError(f'project template target leaves its configuration directory: {target}')
        if not target.exists():
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source, target)
        return target

    def referenced_files(value, base):
        if isinstance(value, dict):
            for item in value.values():
                referenced_files(item, base)
        elif isinstance(value, list):
            for item in value:
                referenced_files(item, base)
        elif isinstance(value, str) and '\n' not in value:
            source = (base / value).resolve()
            try:
                exists = source.is_relative_to(source_root) and source.is_file()
            except OSError:
                exists = False
            if exists:
                copy_file(source)

    def declaration(kind, spec):
        source = paths.find(kind, spec).resolve()
        if not source.is_relative_to(source_root):
            return source
        target = target_root / source.relative_to(source_root)
        if source in visited:
            return target
        visited.add(source)
        data = yaml.safe_load(source.read_text()) or {}
        text = source.read_text()
        if kind == 'benchmarks':
            for key, dependency in (('robot', 'robots'), ('simulator', 'simulators')):
                if data.get(key):
                    child = declaration(dependency, data[key])
                    relative = '../' + str(child.relative_to(target_root))
                    text = re.sub(r'^' + key + r':\s*\S+', key + ': ' + relative, text, flags=re.MULTILINE)
        referenced_files(data, source.parent)
        resources = source.with_suffix('')
        if resources.is_dir():
            for file in sorted(resources.rglob('*')):
                if file.is_file():
                    copy_file(file)
        if target.is_symlink() or not target.resolve().is_relative_to(target_root):
            raise ValueError(f'project template target leaves its configuration directory: {target}')
        if not target.exists():
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_text(text)
        return target

    result = dict(choices)
    benchmark = choices.get('bench')
    if benchmark and not paths.is_path(benchmark):
        source = paths.find('benchmarks', benchmark)
        data = yaml.safe_load(source.read_text()) or {}
        for key, field in (('robot', 'robot'), ('sim', 'simulator')):
            if not result.get(key) and data.get(field):
                result[key] = data[field]
    for key, kind in KINDS.items():
        if result.get(key):
            result[key] = str(declaration(kind, result[key]))
    return result
