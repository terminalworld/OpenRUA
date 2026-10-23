"""Fixed-peer transport handles streaming, half-close and active cleanup."""
import socket
import socketserver
import threading

from openrua.proxy import relay


def test_fixed_peer_relays_binary_data_and_half_close(tmp_path):
    received = []

    class Handler(socketserver.BaseRequestHandler):
        def handle(self):
            parts = []
            while data := self.request.recv(8192):
                parts.append(data)
            received.append(b''.join(parts))
            self.request.sendall(b'reply:' + received[-1])

    upstream = socketserver.ThreadingTCPServer(('127.0.0.1', 0), Handler)
    thread = threading.Thread(target=upstream.serve_forever)
    thread.start()
    try:
        path = tmp_path / 'proxy.sock'
        with relay.tcp_peer(path, *upstream.server_address):
            assert path.stat().st_mode & 0o777 == 0o600
            with relay.relay(('127.0.0.1', 0), lambda: relay._unix_peer(str(path)), unix=False) as front:
                with socket.create_connection(front.server_address, timeout=5) as client:
                    payload = b'CONNECT other-host:1 HTTP/1.1\r\n\r\n' + bytes(range(256)) * 500
                    client.sendall(payload)
                    client.shutdown(socket.SHUT_WR)
                    result = []
                    while data := client.recv(8192):
                        result.append(data)
                    assert b''.join(result) == b'reply:' + payload
        assert received == [payload] and not path.exists()
    finally:
        upstream.shutdown()
        upstream.server_close()
        thread.join()


def test_close_releases_connected_clients(tmp_path):
    accepted = threading.Event()
    left, right = socket.socketpair()

    def connect():
        accepted.set()
        return left

    path = tmp_path / 'relay.sock'
    with relay.relay(str(path), connect, unix=True):
        client = relay._unix_peer(str(path))
        assert accepted.wait(5)
    try:
        client.settimeout(5)
        assert client.recv(1) == b''
        right.settimeout(5)
        assert right.recv(1) == b''
        assert not path.exists()
    finally:
        client.close()
        right.close()
