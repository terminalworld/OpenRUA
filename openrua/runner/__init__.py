"""Compose robot resources for native terminals, shared chat, and trials.

- ``live.py``       one RobotRequest -> live robot and sandbox; explicit shutdown
- ``live_state.py`` retained resource facts, separate from conversation state
- ``managed.py``    live resources + native conversation + shared execution owner
- ``control.py``    local shutdown endpoint for native-terminal resource owners
- ``main.py``       ``openrua bench``: arguments, the run directory, the loop
                    over tasks and seeds
- ``bringup.py``    one resolved config -> sandbox + robot, the same path
                    for native terminals, shared sessions, and trials
- ``trial.py``      one trial: reset, preflight, operator, verdict, record,
                    teardown
- ``operators.py``  who acts on the robot during a trial (none, script,
                    agent) and the table ``openrua bench --operator`` reads
- ``session.py``    the agent operator: a headless agent run as one or
                    more segments of a single session
- ``preflight.py``  every promise the workspace docs make, checked from
                    the sandbox before the agent starts
- ``record.py``     what a trial writes under ``runs/`` (the only writer)
- ``lock.py``       one writer per trial directory

preflight, record and lock are leaves: handles and paths are handed in.
The runner imports every host-side unit and never the bridge.
"""
