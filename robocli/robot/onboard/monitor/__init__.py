"""Monitor: watches the world's every step and answers the host's line.

- ``monitor.py``  the Monitor object: per-step success latch (asks the
                  loader's original predicate after every sim step,
                  remembers the first hit) + the answering verbs
                  (reset / success / task_info / steps, forensic
                  objects / hand)
- ``channel.py``  the stdio line itself: one JSON question per stdin
                  line, one JSON answer per stdout line

Zero benchmark knowledge (the loader arrives as a parameter) and zero
knowledge of the sibling environment and ros_graph packages; boot.py
passes everything in. The measurement-audit history (latch semantics,
episode-scoped step counts, time-to-success stamps) lives here.
"""
