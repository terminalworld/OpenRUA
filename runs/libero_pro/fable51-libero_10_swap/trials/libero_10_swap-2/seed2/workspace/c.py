#!/usr/bin/env python3
"""Client for ctl_server: python3 c.py '{"cmd": "js"}'"""
import json
import socket
import sys

s = socket.create_connection(("127.0.0.1", 5555))
s.sendall((sys.argv[1].strip() + "\n").encode())
data = b""
while not data.endswith(b"\n"):
    chunk = s.recv(65536)
    if not chunk:
        break
    data += chunk
out = json.loads(data.decode())
tb = out.pop("tb", None)
print(json.dumps(out, indent=1))
if tb:
    print(tb)
