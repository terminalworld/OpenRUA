"""Higher-level motion helpers on top of lib/scene: collision-checked IK moves with retry."""
import numpy as np
from lib import *
from scene import *

class Ctl:
    def __init__(self, name="ctl"):
        self.r = Robot(name); self.sc = Scene(self.r.node)

    def ik_valid(self, p, R, seed=None, tries=6, ignore=()):
        """IK solution that passes the validity check (contacts involving `ignore` bodies are tolerated)."""
        seed = list(seed if seed is not None else self.r.arm_q())
        rng = np.random.default_rng(0)
        sols, last = [], None
        for i in range(tries):
            s = seed if i < 2 else list(np.array(seed) + rng.normal(0, 0.15, 7))
            q = self.r.ik(p, R, seed=s)
            if q is None: continue
            v, c = self.sc.check(q, verbose=False)
            c = [x for x in set(map(tuple, c)) if not any(g in x[0] or g in x[1] for g in ignore)]
            if not c:
                sols.append(q)
                if np.abs(np.array(q) - np.array(seed)).max() < 0.5: break
            else: last = c
        if sols:
            return min(sols, key=lambda q: np.abs(np.array(q) - np.array(seed)).max())
        print(f"  ik_valid: no valid IK for {np.round(p,3)}; last contacts {last}", flush=True)
        return None

    def exec(self, qs, times, tol=0.02, retries=3):
        code, err = self.r.move(qs, times)
        for _ in range(retries):
            if err <= tol: break
            code, err = self.r.move([qs[-1]], [max(2.0, times[-1] / 3)])
        if err > tol: raise RuntimeError(f"move did not converge (err {err:.3f})")
        return err

    def goto_q(self, q, t=4.0, ignore=(), steps=25):
        q0 = self.r.arm_q()
        ok, bad = path_valid(self.sc, q0, q, steps=steps)
        bad = [(i, [x for x in c if not any(g in x[0] or g in x[1] for g in ignore)]) for i, c in bad]
        bad = [b for b in bad if b[1]]
        if bad:
            print("  goto_q: path invalid:", bad[:4], flush=True); return False
        self.exec([q], [t]); return True

    def goto(self, p, R, t=4.0, ignore=(), seed=None):
        q = self.ik_valid(p, R, seed=seed, ignore=ignore)
        if q is None: return False
        return self.goto_q(q, t, ignore=ignore)

    def line(self, p1, R, n=6, t=4.0, ignore=(), check=True):
        """Straight Cartesian move from current hand position to p1 at fixed R."""
        p0, _ = self.r.fk()
        qs, q = [], self.r.arm_q()
        for i in range(1, n + 1):
            p = p0 + (np.asarray(p1) - p0) * i / n
            q = self.ik_valid(p, R, seed=q, ignore=ignore) if check else self.r.ik(p, R, seed=q)
            if q is None: print(f"  line: no valid IK at {np.round(p,3)}"); return False
            if np.abs(np.array(q) - np.array(qs[-1] if qs else self.r.arm_q())).max() > 0.6:
                print(f"  line: joint jump at {np.round(p,3)}"); return False
            qs.append(q)
        self.exec(qs, list(np.linspace(t / n, t, n)))
        return True
