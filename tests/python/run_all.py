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
    "test_dummy_stubs_under_tilt",
    "test_wrist_anatomy_constraint",
    "test_thumb_opposition",
    "test_glove_stream_layout",
    "test_per_sensor_asymmetry",
    "test_symyaws2s_guard_consistency",
    "test_metadata_per_plugin",
    "test_pose_hysteresis",
    "test_wrist_constraint_order",
    "test_full_yaw_invariance",
    "test_thumb_gestures",
    "test_heel_toe_gait",
    "test_ui_invariants",
    "test_pose_separation_matrix",
    "test_procrustes_asymmetric_mount",
    "test_z_rate_smoothness",
    "test_palm_with_glove_sensor",
    "test_visual_stream_parity",
    "test_heel_toe_anchor",
    "test_jump_phase",
    "test_toe_dorsiflexion",
    "test_fox_kf_per_segment",
    "test_fox_kf_static_source",
    "test_kf_prior_wiring",
    "test_diff_streams",
    "test_actor_dimensions",
    "test_pose_transition_z",
    "test_finger_anatomy",
    "test_hemisphere_normalization",
    "test_s2s_180_flip",
    "test_foot_sock_orientation",
    "test_pair_symmetry_fallback",
    "test_ecompass_to_triad_fallback",
    "test_static_z_drift",
    "test_finger_adaptive_alpha",
    "test_stream_hemisphere_canonical",
    "test_fox_kf_nan_recovery",
    "test_kf_bias_longterm",
    "test_fox_kf_mag_disable",
    "test_recap_time_based",
    "test_calibration_log_replay",
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
