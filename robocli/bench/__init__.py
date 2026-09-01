"""Running a task set on a robot: ``run`` conducts trials, ``precheck``
verifies every promise the manual makes before the agent boards,
``record`` is the only writer under ``runs/``, ``triallock`` keeps two
conductors off one trial. precheck and record are leaves (handles and
paths are handed in); run imports every host-side unit and never
``robocli.robot.onboard``."""
