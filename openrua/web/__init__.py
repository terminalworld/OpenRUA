"""Browser client assets; the client uses the public session HTTP API only."""

from importlib.resources import files


def assets() -> dict[str, tuple[str, bytes]]:
    root = files(__package__)
    return {route: (mime, root.joinpath(name).read_bytes()) for route, name, mime in (
        ("/", "index.html", "text/html; charset=utf-8"),
        ("/app.js", "app.js", "text/javascript; charset=utf-8"),
        ("/style.css", "style.css", "text/css; charset=utf-8"),
    )}
