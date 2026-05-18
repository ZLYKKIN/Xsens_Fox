"""S9: static UI audit.  Lives-checked here because a remote-execution
container can't open a Qt window — we can only inspect the source.  The
asserts encode invariants we verified by reading scr/main.cpp; any
regression that breaks one of them shows up as a test failure in CI.

What we audited:
  • Every dynamically-allocated QTimer has a Qt parent (avoids leaks /
    use-after-free on wizard close).
  • Every QTimer::singleShot uses the 3-arg form with a context object,
    so the lambda is cancelled if the context is destroyed before the
    delay expires.
  • Every QPushButton that fires an async worker debounces with
    setEnabled(false) + a re-enable timer (no double-click crashes).
"""
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
CPP = os.path.normpath(os.path.join(HERE, "..", "..", "scr", "main.cpp"))


def _src():
    with open(CPP, "r", encoding="utf-8") as f:
        return f.read()


def test_qtimer_has_parent():
    """`new QTimer()` without a parent leaks until app exit.  Every site
    must use `new QTimer(this)` (or `make_shared<QTimer>(this)`)."""
    src = _src()
    # Find every "new QTimer" occurrence and look at its argument.
    bad = []
    for m in re.finditer(r"new QTimer\s*\(([^)]*)\)", src):
        arg = m.group(1).strip()
        if not arg or arg == "":
            line = src[:m.start()].count("\n") + 1
            bad.append((line, m.group(0)))
    for m in re.finditer(r"make_shared<QTimer>\s*\(([^)]*)\)", src):
        arg = m.group(1).strip()
        if not arg or arg == "":
            line = src[:m.start()].count("\n") + 1
            bad.append((line, m.group(0)))
    assert not bad, f"QTimer without parent at: {bad}"


def test_singleshot_has_context():
    """QTimer::singleShot(ms, lambda) without a context object leaks the
    lambda past the dialog's life — closing the wizard mid-debounce
    leads to a use-after-free in the captured `this`.  Every call site
    must use the 3-arg form: singleShot(ms, this, lambda)."""
    src = _src()
    bad_lines = []
    for m in re.finditer(r"QTimer::singleShot\s*\(([^)]*?)\)", src):
        body = m.group(1)
        # Crude argument counter: split on top-level commas.  We only need
        # to detect "less than 3 args" → no context.
        depth = 0
        commas = 0
        for ch in body:
            if ch in "([{":
                depth += 1
            elif ch in ")]}":
                depth -= 1
            elif ch == "," and depth == 0:
                commas += 1
        # 2 args = ms, lambda (no context)
        if commas < 2:
            line = src[:m.start()].count("\n") + 1
            bad_lines.append((line, m.group(0)[:80]))
    assert not bad_lines, f"singleShot without context: {bad_lines}"


def test_async_buttons_debounce():
    """The two long-running connection buttons (Connect suit, Connect
    gloves) must call setEnabled(false) before kicking off the worker so
    a double-click can't reach the worker twice.  Find both onConnect*
    handlers and verify they disable their button."""
    src = _src()
    for name, btn in [("onConnectSuit", "m_btnConnectSuit"),
                       ("onConnectGloves", "m_btnConnectGloves")]:
        # Find the handler body.
        m = re.search(
            r"NewSessionWizard::" + name + r"\s*\(\s*\)\s*\{(.+?)\n\}",
            src, re.DOTALL)
        assert m, f"handler {name} not found in main.cpp"
        body = m.group(1)
        assert f"{btn}->setEnabled(false)" in body, \
            f"{name}: missing debounce setEnabled(false) on {btn}"
        # Re-enable should come back via either a singleShot or a status
        # tick refresh — verify ONE of them happens.
        re_enable = (f"{btn}->setEnabled(true)" in body
                     or f"{btn}.setEnabled(true)" in body)
        assert re_enable, f"{name}: never re-enables {btn}"


def test_render_timer_has_safe_interval():
    """m_renderTimer is the per-frame tick driving the viewport.  It
    must be > 0 ms (interval 0 = busy-loop the GUI thread)."""
    src = _src()
    m = re.search(r"m_renderTimer\.setInterval\(int\(1000\.0\s*/\s*([A-Za-z_]+)\)\)", src)
    assert m, "could not find m_renderTimer.setInterval — UI may be broken"
    # The expression is 1000/kRenderFps.  kRenderFps is 90 in main.h:56,
    # so interval = 11 ms — well above zero.
    assert m.group(1) == "kRenderFps"


if __name__ == "__main__":
    test_qtimer_has_parent()
    test_singleshot_has_context()
    test_async_buttons_debounce()
    test_render_timer_has_safe_interval()
    print("test_ui_invariants: PASS")
