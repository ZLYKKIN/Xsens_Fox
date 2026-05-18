"""Run every Python regression test in this directory.  Returns non-zero
on the first failure so `build.bat` (or any CI step) can gate on this.

Usage:
    python tests/python/run_all.py
"""
import importlib, os, sys, traceback

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

TEST_MODULES = [
    "test_default_angles",
    "test_global_yaw",
    "test_symmetry",
    "test_locomotion_thresholds",
    "test_shoulder_cone",
    "test_stream_frame",
]


def main():
    failed = []
    for name in TEST_MODULES:
        try:
            m = importlib.import_module(name)
        except Exception:
            print(f"[IMPORT FAIL] {name}")
            traceback.print_exc()
            failed.append(name)
            continue
        funcs = [(k, v) for k, v in vars(m).items()
                 if k.startswith("test_") and callable(v)]
        for fname, fn in funcs:
            try:
                fn()
            except AssertionError as e:
                print(f"[FAIL] {name}.{fname}: {e}")
                failed.append(f"{name}.{fname}")
            except Exception as e:
                print(f"[ERROR] {name}.{fname}: {e}")
                traceback.print_exc()
                failed.append(f"{name}.{fname}")
        print(f"  {name}: {len(funcs)} test(s)")

    print()
    if failed:
        print(f"FAILED: {len(failed)}")
        for f in failed:
            print(f"  - {f}")
        sys.exit(1)
    print("ALL TESTS PASSED")


if __name__ == "__main__":
    main()
