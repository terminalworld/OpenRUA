"""Relay a fixed TCP peer over a Unix socket, or expose that socket on loopback.

The peer is an explicit parameter. This module neither interprets HTTP nor
chooses destinations from client data; the upstream proxy enforces its policy.
"""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import json
import math
import os
from pathlib import Path
import selectors
import socket
import socketserver
import threading


def _copy(left: socket.socket, right: socket.socket) -> None:
    with selectors.DefaultSelector() as selector:
        selector.register(left, selectors.EVENT_READ, right)
        selector.register(right, selectors.EVENT_READ, left)
        while selector.get_map():
            for key, _ in selector.select():
                data = key.fileobj.recv(65536)
                if data:
                    key.data.sendall(data)
                else:
                    selector.unregister(key.fileobj)
                    try:
                        key.data.shutdown(socket.SHUT_WR)
                    except OSError:
                        pass


class _UnixServer(socketserver.ThreadingUnixStreamServer):
    daemon_threads = True
    block_on_close = False


class _TCPServer(socketserver.ThreadingTCPServer):
    daemon_threads = True
    block_on_close = False


@contextmanager
def relay(address, connect, *, unix: bool):
    """A running threaded relay; closing it also closes active connections."""
    connections: set[socket.socket] = set()
    lock = threading.Lock()
    stopping = threading.Event()

    class Handler(socketserver.BaseRequestHandler):
        def handle(self):
            peer = None
            try:
                with lock:
                    if stopping.is_set():
                        return
                    connections.add(self.request)
                peer = connect()
                with lock:
                    if stopping.is_set():
                        return
                    connections.add(peer)
                _copy(self.request, peer)
            except (OSError, ValueError):
                # A closed client or deadline is an ordinary transport end.
                pass
            finally:
                with lock:
                    connections.discard(self.request)
                    if peer is not None:
                        connections.discard(peer)
                if peer is not None:
                    peer.close()

    server = (_UnixServer if unix else _TCPServer)(address, Handler)
    if unix:
        os.chmod(address, 0o600)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield server
    finally:
        stopping.set()
        server.shutdown()
        server.server_close()
        with lock:
            for connection in connections:
                try:
                    connection.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                connection.close()
        thread.join()
        if unix:
            Path(address).unlink(missing_ok=True)


@contextmanager
def tcp_peer(path: Path, host: str, port: int):
    """Make only this TCP peer reachable through a private Unix socket."""
    def connect():
        peer = socket.create_connection((host, port), timeout=10)
        peer.settimeout(None)
        return peer
    with relay(str(path), connect, unix=True) as server:
        yield server


def _unix_peer(path: str):
    peer = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        peer.connect(path)
    except BaseException:
        peer.close()
        raise
    return peer


def forward(path: Path, ready: Path, seconds: float) -> None:
    """Serve a Unix peer on an ephemeral loopback port for a bounded lifetime."""
    if not math.isfinite(seconds) or seconds <= 0:
        raise ValueError('relay lifetime must be positive')
    with relay(('127.0.0.1', 0), lambda: _unix_peer(str(path)), unix=False) as server:
        ready.write_text(json.dumps({'host': '127.0.0.1', 'port': server.server_address[1]}) + '\n')
        threading.Event().wait(seconds)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    actions = parser.add_subparsers(dest='action', required=True)
    front = actions.add_parser('forward', help='expose a Unix socket on a temporary loopback port')
    front.add_argument('--socket', type=Path, required=True)
    front.add_argument('--ready', type=Path, required=True)
    front.add_argument('--seconds', type=float, required=True)
    args = parser.parse_args()
    forward(args.socket, args.ready, args.seconds)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
