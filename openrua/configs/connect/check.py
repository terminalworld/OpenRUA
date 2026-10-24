#!/usr/bin/env python3
"""Ask OpenRUA, on the host, to check a step of connecting this robot.

Usage: python3 /openrua-connect/check.py STEP

STEP is one of:
  host     this computer: docker, disk, real-time limits, devices
  config   robot.yaml loads and has no TODO left
  drivers  every image robot.yaml builds (backend.build) builds
  probe    reads the running ROS 2 graph into robot.draft.yaml
  robot    the robot comes up twice from robot.yaml and matches it
  done     every check passed: the session moves to the robot's own terminal

It waits for the result and prints it; the full output is in
.openrua/connect/<session>/<step>.log in the project. Exit status 0 means passed.
"""
import json
import sys
import time
import urllib.error
import urllib.request

STEPS = ("host", "config", "drivers", "probe", "robot", "done")


def call(endpoint, path, body=None):
    request = urllib.request.Request(
        endpoint["url"] + path, data=None if body is None else json.dumps(body).encode(),
        headers={"Authorization": "Bearer " + endpoint["token"],
                 "Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(request, timeout=60) as answer:
            return json.load(answer)
    except urllib.error.HTTPError as error:
        raise SystemExit(json.load(error).get("error", str(error)))


def main():
    if len(sys.argv) != 2 or sys.argv[1] not in STEPS:
        raise SystemExit(__doc__)
    step = sys.argv[1]
    with open("/openrua-connect/endpoint.json") as stream:
        endpoint = json.load(stream)
    started = call(endpoint, "/api/actions/check", {"step": step})["result"]
    if step == "done":
        print(started["summary"])
        return 0 if started["status"] == "passed" else 1
    print(f"[check] {step}: running; output under .openrua/connect/<session>/{step}.log in the project", flush=True)
    while True:
        state = call(endpoint, "/api/session")["state"]
        result = state.get("checks", {}).get(step, {})
        if result.get("status") in ("passed", "failed") and result.get("run") == started["run"]:
            print(f"[check] {step}: {result['status']}\n{result['summary']}")
            return 0 if result["status"] == "passed" else 1
        time.sleep(3)


if __name__ == "__main__":
    sys.exit(main())
