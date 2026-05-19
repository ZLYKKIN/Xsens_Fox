"""F-4.2 — simulates the LiveStreamSender recap state machine.  At 30 fps
the original frame-counter approach is fine, but if real frame delivery
drops to ~15 Hz (which can happen when receiver thread stalls), the
frame counter (target = fps*5) won't expire for ten real seconds.  The
time-based safety net forces recap to close after 5 s of wall time
regardless of frame rate."""


class RecapSim:
    def __init__(self, declared_fps, deadline_ms=5000):
        self.target_frames = max(60, declared_fps * 5)
        self.frames_left = self.target_frames
        self.deadline_ms = deadline_ms
        self.started_at_ms = 0
        self.committed = False

    def step(self, now_ms):
        if self.frames_left <= 0:
            return
        self.frames_left -= 1
        deadline_hit = (now_ms - self.started_at_ms) >= self.deadline_ms
        if self.frames_left == 0 or deadline_hit:
            self.committed = True
            self.frames_left = 0


def test_recap_completes_within_5s_at_full_fps():
    sim = RecapSim(declared_fps=30)
    t = 0
    while not sim.committed and t < 20000:
        sim.step(t); t += int(1000 / 30)
    assert sim.committed and t <= 5500, f"recap not closed in 5s, t={t}ms"


def test_recap_completes_within_5s_under_frame_drop():
    sim = RecapSim(declared_fps=30)
    t = 0
    while not sim.committed and t < 20000:
        sim.step(t); t += int(1000 / 15)
    assert sim.committed and t <= 5500, (
        f"under frame drop recap still must close in ~5s; t={t}ms")


def test_recap_does_not_close_prematurely():
    sim = RecapSim(declared_fps=30)
    sim.step(100)
    assert not sim.committed
    sim.step(500)
    assert not sim.committed
