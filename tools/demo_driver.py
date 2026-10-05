"""Drives the simulator through real X11 mouse/keyboard input (xdotool) while it
renders a fixed 60 fps timeline into a video (psim --record).

The app writes its frame number to a status file every frame; the driver waits
for frame numbers before each action, so the recording is deterministic no
matter how fast the machine renders.

Requires: Xvfb, xdotool, ffmpeg. Used by tools/record_demo.py.
"""
import os
import subprocess
import time


class Driver:
    def __init__(self, psim, out, width=1600, height=900, display=":99", scene=0, work_dir="/tmp/psim-demo", extra=()):
        self.display = display
        self.env = dict(os.environ, DISPLAY=display)
        os.makedirs(work_dir, exist_ok=True)
        self.status = os.path.join(work_dir, "frame.txt")
        self.caption_file = os.path.join(work_dir, "caption.txt")
        for f in (self.status, self.caption_file):
            open(f, "w").close()
        self.xvfb = subprocess.Popen(["Xvfb", display, "-screen", "0", f"{width}x{height}x24", "-nolisten", "tcp"],
                                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(1.0)
        cmd = [psim, "--size", f"{width}x{height}", "--scene", str(scene), "--record", out,
               "--status-file", self.status, "--caption-file", self.caption_file, *extra]
        self.app = subprocess.Popen(cmd, env=self.env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.frame_now = 0
        self.wait(5)
        win = self._xdo("search", "--sync", "--name", "Particle Simulator").split()
        if win:
            self._xdo("windowfocus", "--sync", win[0])

    # -- low level -----------------------------------------------------------
    def _xdo(self, *args):
        return subprocess.run(["xdotool", *map(str, args)], env=self.env, capture_output=True, text=True).stdout

    def current_frame(self):
        try:
            with open(self.status) as f:
                return int(f.read().strip() or 0)
        except (OSError, ValueError):
            return 0

    def wait(self, frames):
        """Waits until `frames` more frames have been rendered."""
        target = self.frame_now + frames
        while True:
            if self.app.poll() is not None:
                raise RuntimeError("psim exited early")
            f = self.current_frame()
            if f >= target:
                self.frame_now = target
                return
            time.sleep(0.002)

    def seconds(self, s):
        self.wait(int(round(s * 60)))

    # -- actions ----------------------------------------------------------------
    def caption(self, text):
        with open(self.caption_file, "w") as f:
            f.write(text)

    def move(self, x, y, frames=0, start=None):
        """Moves the mouse to (x, y); with frames > 0 glides there over that many frames."""
        if frames > 0 and start is not None:
            sx, sy = start
            for k in range(1, frames + 1):
                t = k / frames
                t = t * t * (3 - 2 * t)  # ease in/out
                self._xdo("mousemove", int(sx + (x - sx) * t), int(sy + (y - sy) * t))
                self.wait(1)
        else:
            self._xdo("mousemove", int(x), int(y))
            self.wait(1)
        self.pos = (x, y)

    def glide(self, x, y, frames=20):
        self.move(x, y, frames, getattr(self, "pos", (x, y)))

    def click(self, x, y, glide_frames=18, hold=3):
        self.glide(x, y, glide_frames)
        self._xdo("mousedown", 1)
        self.wait(hold)
        self._xdo("mouseup", 1)
        self.wait(2)

    def press(self):
        self._xdo("mousedown", 1)
        self.wait(1)

    def release(self):
        self._xdo("mouseup", 1)
        self.wait(1)

    def drag(self, path, frames_per_leg=20, hold_end=0):
        """Presses at path[0], glides through the remaining points, releases."""
        self.glide(*path[0])
        self.press()
        for p in path[1:]:
            self.glide(p[0], p[1], frames_per_leg)
        self.wait(hold_end)
        self.release()

    def key(self, name, after=6):
        # raylib samples key state once per frame: hold the key across frames
        self._xdo("keydown", name)
        self.wait(2)
        self._xdo("keyup", name)
        self.wait(after)

    def finish(self):
        """Quits with the app's own key (Q) so it closes the video file cleanly."""
        self.caption("")
        self.wait(2)
        self._xdo("keydown", "q")
        time.sleep(0.5)
        self._xdo("keyup", "q")
        try:
            self.app.wait(timeout=120)
        except subprocess.TimeoutExpired:
            self.app.kill()
        self.xvfb.terminate()
