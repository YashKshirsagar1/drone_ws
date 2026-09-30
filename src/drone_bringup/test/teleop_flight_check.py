"""Drive teleop_key through a pty and check the drone actually responds.

An integration check, not a unit test - it needs the sim already running, so
it is deliberately not named `test_*` and pytest will not collect it:

    ros2 launch drone_bringup drone_sim.launch.py headless:=true &
    python3 src/drone_bringup/test/teleop_flight_check.py

The node reads single keypresses from a terminal, so a pipe will not do: it
needs a pty on stdin. Keys are sent repeatedly to stand in for the keyboard
auto-repeat that holding a key down produces.

It flies the whole keyboard: `t` to take off, each arrow, `w`, the decay back
to a hover once keys stop, then `l` to land and `q` to quit.
"""
import os
import pty
import select
import subprocess
import time

FAIL = []


def check(label, ok, detail=''):
    print(f'{"PASS" if ok else "FAIL"}  {label}  {detail}')
    if not ok:
        FAIL.append(label)


def position():
    """Current /odom position as (x, y, z)."""
    out = subprocess.run(
        ['ros2', 'topic', 'echo', '/odom', '--once', '--field',
         'pose.pose.position'],
        capture_output=True, text=True, timeout=20).stdout
    vals = {}
    for line in out.splitlines():
        if ':' in line:
            k, _, v = line.partition(':')
            try:
                vals[k.strip()] = float(v)
            except ValueError:
                pass
    return vals.get('x'), vals.get('y'), vals.get('z')


master, slave = pty.openpty()
proc = subprocess.Popen(['ros2', 'run', 'drone_bringup', 'teleop_key'],
                        stdin=slave, stdout=slave, stderr=slave)
os.close(slave)
log = ''


def pump(seconds, keys=b''):
    """Feed `keys` at 20 Hz for `seconds` while collecting the node's output."""
    global log
    end = time.time() + seconds
    while time.time() < end:
        if keys:
            os.write(master, keys)
        r, _, _ = select.select([master], [], [], 0.05)
        if r:
            try:
                log += os.read(master, 8192).decode(errors='ignore')
            except OSError:
                break
    return log


try:
    # Press it immediately, with no warm-up. This is the case that matters: a
    # ROS 2 publisher drops what it sends before its subscriber is matched, so
    # arming on the very first tick used to be lost and the drone then ignored
    # every command that followed. Idling first hides the bug completely.
    os.write(master, b't')
    pump(25)
    check('prints its key map on start', 'quadcopter teleop' in log)
    check('t arms the motors even when pressed instantly',
          'motors armed' in log)
    check('t reports takeoff', 'taking off to 3.00 m' in log)
    check('t reaches and holds the target', 'holding' in log,
          [l for l in log.splitlines() if 'holding' in l])
    x0, y0, z0 = position()
    check('hovers near 3 m above rest', z0 is not None and abs(z0 - 3.04) < 0.3,
          f'z={z0:.3f}')

    # Right arrow - body -y, and with yaw ~0 that is world -y.
    os.write(master, b'\x1b[C')
    pump(4, b'\x1b[C')
    pump(2)
    x1, y1, z1 = position()
    check('right arrow moves it right (-y)', y1 < y0 - 0.5,
          f'y {y0:.2f} -> {y1:.2f}')

    # w - forward, body +x.
    pump(4, b'w')
    pump(2)
    x2, y2, z2 = position()
    check('w moves it forward (+x)', x2 > x1 + 0.5, f'x {x1:.2f} -> {x2:.2f}')

    # Releasing the keys should decay to a hover, not keep drifting.
    pump(3)
    x3, y3, z3 = position()
    pump(2)
    x4, y4, z4 = position()
    drift = ((x4 - x3) ** 2 + (y4 - y3) ** 2) ** 0.5
    check('decays to a hover once keys stop', drift < 0.25, f'drift={drift:.3f} m')

    # Up / down arrows.
    pump(3, b'\x1b[A')
    pump(2)
    _, _, z5 = position()
    check('up arrow climbs', z5 > z4 + 0.3, f'z {z4:.2f} -> {z5:.2f}')
    pump(3, b'\x1b[B')
    pump(2)
    _, _, z6 = position()
    check('down arrow descends', z6 < z5 - 0.3, f'z {z5:.2f} -> {z6:.2f}')

    # l - land and disarm. Pressed while still moving forward, on purpose: the
    # auto modes take over the vertical command, and the hover decay only runs
    # in manual mode, so a stale horizontal velocity would ride out the whole
    # descent and land it somewhere else entirely.
    pump(1.5, b'w')
    os.write(master, b'l')
    xl, yl, _ = position()
    # Held down, the way a terminal auto-repeats a key you keep pressed. Each
    # repeat used to reset the touchdown detector, so the drone descended for
    # ever and never reported landing.
    pump(30, b'l')
    xe, ye, _ = position()
    carried = ((xe - xl) ** 2 + (ye - yl) ** 2) ** 0.5
    check('l lands straight down, not still sliding', carried < 1.5,
          f'moved {carried:.2f} m horizontally while landing')
    check('l reports landing', 'landing' in log)
    check('l detects touchdown', 'touchdown' in log,
          [l for l in log.splitlines() if 'touchdown' in l])
    check('l disarms after landing', 'motors disarmed' in log)
    _, _, z7 = position()
    check('ends up on the ground', z7 is not None and z7 < 0.2, f'z={z7:.3f}')

    # q - quit cleanly.
    os.write(master, b'q')
    pump(5)
    proc.wait(timeout=15)
    check('q exits cleanly', proc.returncode == 0, f'rc={proc.returncode}')
    check('no traceback', 'Traceback' not in log)

finally:
    if proc.poll() is None:
        proc.kill()
    print('\n----- node output -----')
    print(log[-3000:])

print('\nRESULT:', 'ALL TELEOP CHECKS PASSED' if not FAIL else f'FAILURES: {FAIL}')
