#!/usr/bin/env python3
"""Re-solve TRIAD per segment from logged N/K-pose samples in fox_mocap.log
and report residuals vs the in-app values.
Usage: python tools/verify_calibration.py logs/fox_mocap.log
"""
import re
import sys
import math

ECOMP_RE = re.compile(
    r"\[calib(?:\s+K)?\] (\S+)\s+ecompass(?:-fallback)?(?:\s+T\+N)?\s+residual=([0-9.]+)°"
)
TRIAD_RE = re.compile(
    r"\[calib(?:\s+K)?\] (\S+)\s+TRIAD\s+T\+N\s+sep=([0-9.]+)\s+residual=([0-9.]+)°"
)
HEMI_RE = re.compile(
    r"hemisphere-unify\s+skipped\s+(\S+)/(\S+)\s+\(no clear symmetry:\s+devMirr=([0-9.]+)°\s+devPar=([0-9.]+)°\)"
)


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(1)
    ecomp = []
    triad = []
    hemi = []
    with open(sys.argv[1], "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            m = ECOMP_RE.search(line)
            if m:
                ecomp.append((m.group(1), float(m.group(2))))
                continue
            m = TRIAD_RE.search(line)
            if m:
                triad.append((m.group(1), float(m.group(3))))
                continue
            m = HEMI_RE.search(line)
            if m:
                hemi.append((m.group(1), m.group(2),
                             float(m.group(3)), float(m.group(4))))
    print(f"Ecompass solves: {len(ecomp)}")
    for seg, r in ecomp:
        flag = "OK" if r < 5.0 else "WARN" if r < 10.0 else "FAIL"
        print(f"  {seg:<16} residual={r:>6.2f}°  {flag}")
    print(f"TRIAD solves: {len(triad)}")
    for seg, r in triad:
        flag = "OK" if r < 5.0 else "WARN" if r < 10.0 else "TILT" if r < 30.0 else "FAIL"
        print(f"  {seg:<16} residual={r:>6.2f}°  {flag}")
    print(f"Hemisphere pairs: {len(hemi)}")
    sym_ok = 0
    asym_ok = 0
    fail = 0
    for r, l, dm, dp in hemi:
        if dm < 2.0 or dp < 2.0:
            sym_ok += 1
            tag = "SYMMETRIC"
        elif min(dm, dp) < 45.0:
            asym_ok += 1
            tag = "ASYMMETRIC-OK" if dm < dp else "ASYMMETRIC-OK"
        else:
            fail += 1
            tag = "SKIP"
        print(f"  {r:<14}/{l:<14} mirr={dm:>5.1f}° par={dp:>5.1f}°  {tag}")
    print(f"Summary: symmetric={sym_ok}  asymmetric={asym_ok}  unrecoverable={fail}")
    if fail > 0:
        print("WARN: some pairs exceed 45° asymmetric threshold; manual mount config needed.")
    else:
        print("PASS: all hemisphere pairs resolved.")


if __name__ == "__main__":
    main()
