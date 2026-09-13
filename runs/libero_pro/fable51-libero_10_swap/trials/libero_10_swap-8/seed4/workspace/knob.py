"""Knob grasp pick-and-place: pinch the lid knob from above, pot hangs plumb.
Usage: python3 knob.py <pot: A|B> [dry]"""
import sys
import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from robot import *
from pp import PP

PITCH = -0.40                    # hand leans toward the robot (extends reach)
KNOB_Z = 1.044                   # fingertip (TCP) height for the pinch: knob spans ~1.041-1.053
KNOB_TOP_DROP = 0.009            # pinch = measured knob top - this
HOVER_Z = 1.25
CARRY_Z = 1.20
CARRY_END_Z = 1.14               # above the target: pot bottom ~0.98, clears the dial (0.959)
PLACE_DZ = 0.040                 # release TCP = pinch height + this: pot bottom ~0.94, 1 cm above the burner

POTS = {  # knob centre from the birdview cloud, and target on the stove
    'A': dict(knob=(-0.1965, -0.201), target=(0.22, -0.01), pick_pitch=float(sys.argv[3]) if len(sys.argv) > 3 else 0.0),
    'B': dict(knob=(-0.0425, 0.255), target=(0.16, 0.085), pick_pitch=PITCH),
}


def R_of(pitch=PITCH):
    return grasp_R(0.0, pitch)


class Knob(PP):
    def __init__(self):
        super().__init__()
        self.tfbuf = Buffer()
        TransformListener(self.tfbuf, self.r.node)

    # ---- IK along a path with a fixed orientation, chained seeds -------
    def path(self, tcps, R, seed):
        q = np.asarray(seed)
        out = []
        Rs = R if isinstance(R, list) else [R] * len(tcps)
        for p, R in zip(tcps, Rs):
            s = self.r.ik_tcp(p, R, seed=q, timeout=0.5)
            if s is None or np.abs(s - q).max() > 0.6:
                s2 = self.r.ik_best(p, R, prev=q, n_random=40)
                if s2 is not None and (s is None or np.abs(s2 - q).max() < np.abs(s - q).max()):
                    s = s2
            if s is None:
                raise RuntimeError(f'IK failed at {np.round(p, 3)}')
            out.append(s)
            q = s
        return out

    VMAX = 0.15                     # rad/s; controller tracks ~0.2 rad/s at most

    def exec(self, qs, total_t, label=''):
        # time each segment by its largest joint step so the controller can track
        q0 = self.r.arm_q()
        steps = [np.abs(b - a).max() for a, b in zip([q0] + list(qs[:-1]), qs)]
        segs = [max(0.3, st / self.VMAX) for st in steps]
        scale = max(1.0, total_t / sum(segs))
        ts = list(np.cumsum(segs) * scale)
        code = self.r.move(qs, ts)
        err = np.abs(self.r.arm_q() - qs[-1]).max()
        print(f'  [{label}] {len(qs)} pts in {ts[-1]:.1f}s code={code} final joint err={err:.4f}')
        tries = 0
        while (code != 0 or err > 0.02) and tries < 4:
            tries += 1
            code = self.r.move([qs[-1]], [max(1.0, err / self.VMAX)])
            err = np.abs(self.r.arm_q() - qs[-1]).max()
            print(f'  [{label}] retry {tries} code={code} err={err:.4f}')
        return code, err

    def settle(self, t=1.0):
        q = self.r.arm_q()
        self.r.move([q], [t])

    def go_line(self, p1, R, t, label, dry=False, seed=None, pitch_from=None, pitch_to=None):
        p0 = self.tcp_now()[0] if seed is None else self.tcp_of(seed)
        pts = self.line(p0, p1)
        if pitch_from is not None:      # interpolate the hand pitch along the line
            n = len(pts)
            R = [R_of(pitch_from + (pitch_to - pitch_from) * (i + 1) / n) for i in range(n)]
        qs = self.path(pts, R, self.r.arm_q() if seed is None else seed)
        jump = max(np.abs(np.diff(np.array([qs[0] if seed is None else seed] + qs), axis=0)).max(1))
        print(f'  {label}: {len(qs)} pts, max step {jump:.2f}, last q {np.round(qs[-1], 2)}')
        if not dry:
            self.exec(qs, t, label)
        return qs[-1]

    def tcp_of(self, q, R=None):
        p, Rq = self.r.fk(q)
        return p + TCP * Rq[:, 2]

    # ---- eye-in-hand cloud -------------------------------------------
    def grab(self, topic, typ, timeout=15.0):
        got = {}
        sub = self.r.node.create_subscription(typ, topic, lambda m: got.setdefault('m', m), 1)
        import time
        t0 = time.time()
        while 'm' not in got and time.time() - t0 < timeout:
            rclpy.spin_once(self.r.node, timeout_sec=0.2)
        self.r.node.destroy_subscription(sub)
        return got.get('m')

    def eih_cloud(self):
        cam = 'robot0_eye_in_hand'
        d = self.grab(f'/{cam}/depth/image_raw', Image)
        info = self.grab(f'/{cam}/color/camera_info', CameraInfo)
        depth = np.frombuffer(d.data, dtype=np.float32).reshape(d.height, d.width)
        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
        v, u = np.mgrid[0:d.height, 0:d.width]
        z = depth
        ok = np.isfinite(z) & (z > 0.05)
        pc = np.stack([(u - cx) * z / fx, (v - cy) * z / fy, z], -1)[ok]
        frame = f'{cam}_optical_frame'
        import time
        t0 = time.time()
        while time.time() - t0 < 10 and not self.tfbuf.can_transform('world', frame, rclpy.time.Time()):
            rclpy.spin_once(self.r.node, timeout_sec=0.2)
        t = self.tfbuf.lookup_transform('world', frame, rclpy.time.Time())
        q = t.transform.rotation
        R = quat_R([q.x, q.y, q.z, q.w])
        tr = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
        return pc @ R.T + tr

    def find_knob(self, guess, rad=0.05):
        P = self.eih_cloud()
        tcp_z = self.tcp_now()[0][2]
        # drop the gripper's own fingers (near the lens) and anything not pot-top height
        P = P[(P[:, 2] < tcp_z - 0.05) & (P[:, 2] > 1.04) & (P[:, 2] < 1.10)]
        near = P[(np.abs(P[:, 0] - guess[0]) < rad) & (np.abs(P[:, 1] - guess[1]) < rad)]
        if len(near) < 20:
            print('  too few knob points near guess', len(near)); return None
        zmax = near[:, 2].max()
        top = near[near[:, 2] > zmax - 0.006]
        c = top[:, :2].mean(0)
        ext = np.ptp(top[:, 0]), np.ptp(top[:, 1])
        print(f'  knob: zmax={zmax:.4f} n_top={len(top)} centre={np.round(c, 4)} '
              f'extent {np.round(ext, 3)}  (birdview guess {guess})')
        if max(ext) > 0.03 or np.linalg.norm(c - np.asarray(guess)) > 0.025:
            print('  refinement rejected, keeping the guess'); return None
        return c, zmax

    # ---- the routine ---------------------------------------------------
    def run(self, name, dry=False):
        pot = POTS[name]
        Rp = R_of(pot['pick_pitch'])
        R = R_of()
        kx, ky = pot['knob']
        tx, ty = pot['target']
        q0 = self.r.arm_q()
        print('start q', np.round(q0, 2), 'gap', round(self.r.finger_gap(), 4))
        if not dry:
            print('open', self.r.gripper(0.04))
        hover = np.array([kx, ky, HOVER_Z])
        # move to hover via a single IK (free-space move)
        qh = self.r.ik_best(hover, Rp, prev=q0, n_random=40)
        if qh is None:
            raise RuntimeError('no IK for hover')
        print('  hover q', np.round(qh, 2))
        res = None
        if not dry:
            self.exec([qh], 4.0, 'hover')
            res = self.find_knob((kx, ky), rad=0.03)
            if res is not None:
                (kx, ky), zmax = res
                # re-hover exactly above the refined knob
                self.go_line([kx, ky, HOVER_Z], Rp, 1.5, 'rehover')
        seed = qh
        knob_z = KNOB_Z
        if not dry and res is not None:
            knob_z = zmax - KNOB_TOP_DROP
        print(f'  pinch height {knob_z:.4f}')
        # descend
        g = np.array([kx, ky, knob_z])
        seed = self.go_line(g, Rp, 4.0, 'descend', dry, seed if dry else None)
        if not dry:
            p = self.tcp_now()[0]; print('  tcp at pinch', np.round(p, 4))
            print('close', self.r.gripper(0.0))
            self.settle(1.0)
            gap = self.r.finger_gap(); print(f'  gap after close {gap:.4f}')
            if gap < 0.004:
                print('GRASP FAILED (gap too small); opening and stopping')
                self.r.gripper(0.04)
                self.go_line([kx, ky, HOVER_Z], Rp, 3.0, 'abort-up')
                return False
        # lift
        seed = self.go_line([kx, ky, CARRY_Z], Rp, 3.0, 'lift', dry, seed if dry else None)
        if not dry:
            gap = self.r.finger_gap(); print(f'  gap after lift {gap:.4f}')
        # carry
        seed = self.go_line([tx, ty, CARRY_END_Z], R, 8.0, 'carry', dry, seed if dry else None,
                            pitch_from=pot['pick_pitch'], pitch_to=PITCH)
        # lower
        seed = self.go_line([tx, ty, knob_z + PLACE_DZ], R, 4.0, 'lower', dry, seed if dry else None)
        if not dry:
            print('open', self.r.gripper(0.04))
        # retreat straight up
        seed = self.go_line([tx, ty, CARRY_END_Z], R, 3.0, 'retreat', dry, seed if dry else None)
        return True


if __name__ == '__main__':
    name = sys.argv[1]
    dry = len(sys.argv) > 2 and sys.argv[2] == 'dry'
    k = Knob()
    ok = k.run(name, dry)
    print('done', ok)
