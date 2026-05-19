"""Real values extracted from logs/fox_mocap.log — used as test fixtures.

Each constant is annotated with the line number in the original log where
the value was observed.  Numbers are reproduced as-is (no rounding).
"""

CALIB_TN_RESIDUAL = {
    "pelvis":      ("ecompass", 0.00),
    "t8":          ("ecompass", 0.00),
    "head":        ("ecompass", 0.00),
    "r_shoulder":  ("ecompass", 0.00),
    "r_upper_arm": ("triad",    7.11),
    "r_forearm":   ("triad",   11.11),
    "r_hand":      ("triad",   14.73),
    "l_shoulder":  ("ecompass", 0.00),
    "l_upper_arm": ("triad",    7.74),
    "l_forearm":   ("triad",    2.50),
    "l_hand":      ("triad",   13.11),
    "r_upper_leg": ("tilt",    55.47),
    "r_lower_leg": ("tilt",    50.86),
    "r_foot":      ("tilt",    85.53),
    "l_upper_leg": ("tilt",    55.22),
    "l_lower_leg": ("tilt",    77.31),
    "l_foot":      ("tilt",    83.75),
}

S2S_ROTATION_DEG = {
    "r_upper_arm": 177.5,
    "r_forearm":   177.4,
    "r_hand":      178.6,
    "l_upper_arm": 174.4,
    "l_forearm":   169.3,
    "l_hand":      178.6,
    "r_upper_leg": 162.7,
    "r_lower_leg": 159.6,
    "r_foot":      137.2,
    "l_upper_leg": 161.1,
    "l_lower_leg": 173.1,
    "l_foot":      138.1,
}

CALIB_K_RESULT = {
    "r_upper_arm": ("wahba", 7.25),
    "r_forearm":   ("triad_fallback", 11.11),
    "r_hand":      ("wahba", 11.96),
    "l_upper_arm": ("wahba", 9.35),
    "l_forearm":   ("triad_fallback", 2.50),
    "l_hand":      ("triad_fallback", 12.56),
    "r_upper_leg": ("triad_fallback", 1.51),
    "r_lower_leg": ("tilt", 50.86),
    "r_foot":      ("tilt", 85.53),
    "l_upper_leg": ("triad_fallback", 3.26),
    "l_lower_leg": ("tilt", 77.31),
    "l_foot":      ("tilt", 83.75),
}

CONFIDENCE = {
    "pelvis":      ("ecomp", 0.85,  0.00),
    "t8":          ("ecomp", 0.85,  0.00),
    "head":        ("ecomp", 0.83,  0.00),
    "r_shoulder":  ("ecomp", 0.00,  0.00),
    "r_upper_arm": ("triad", 0.00,  7.25),
    "r_forearm":   ("triad", 0.00, 11.11),
    "r_hand":      ("triad", 0.37, 11.96),
    "l_shoulder":  ("ecomp", 0.00,  0.00),
    "l_upper_arm": ("triad", 0.00,  9.35),
    "l_forearm":   ("triad", 0.00,  2.50),
    "l_hand":      ("triad", 0.37, 12.56),
    "r_upper_leg": ("triad", 0.88,  1.51),
    "r_lower_leg": ("ecomp", 0.00, 50.86),
    "r_foot":      ("ecomp", 0.00, 85.53),
    "l_upper_leg": ("triad", 0.87,  3.26),
    "l_lower_leg": ("ecomp", 0.00, 77.31),
    "l_foot":      ("ecomp", 0.00, 83.75),
}

PAIR_SYMMETRY = {
    ("r_shoulder",  "l_shoulder"):  dict(rw=-0.501, lw=-0.837, devMirr=76.74,  devPar=148.58),
    ("r_upper_arm", "l_upper_arm"): dict(rw= 0.01,  lw= 0.00,  devMirr=31.95,  devPar= 40.47),
    ("r_forearm",   "l_forearm"):   dict(rw= 0.02,  lw= 0.09,  devMirr=48.15,  devPar= 35.86),
    ("r_hand",      "l_hand"):      dict(rw= 0.03,  lw=-0.12,  devMirr=18.92,  devPar=167.28),
    ("r_upper_leg", "l_upper_leg"): dict(rw= 0.13,  lw= 0.13,  devMirr= 0.00,  devPar=159.29),
    ("r_lower_leg", "l_lower_leg"): dict(rw= 0.18,  lw= 0.06,  devMirr=75.29,  devPar= 20.09),
    ("r_foot",      "l_foot"):      dict(rw= 0.36,  lw= 0.36,  devMirr=55.09,  devPar= 93.91),
}

TPOSE_QUAT_FUSED = {
    "r_upper_leg": (0.343, -0.777, -0.001,  0.528),
    "r_lower_leg": (-0.377, 0.161,  0.502,  0.761),
    "r_foot":      (-0.264, 0.199, -0.522, -0.786),
    "l_upper_leg": ( 0.111, 0.591,  0.084, -0.795),
    "l_lower_leg": ( 0.422, 0.294, -0.843,  0.156),
    "l_foot":      (-0.467,-0.718,  0.480,  0.192),
}

TPOSE_ACC_FOOT = {
    "r_foot": (-0.541,  0.101,  0.588),
    "l_foot": (-0.498, -0.409,  0.735),
}

TPOSE_LR_PAIR_ANGLE_DEG = {
    "shoulder":   dict(L=115.57, R=163.98, dLR=-48.42),
    "upper_arm":  dict(L=150.02, R=160.29, dLR=-10.28),
    "forearm":    dict(L=176.30, R=174.16, dLR=  2.14),
    "hand":       dict(L=123.93, R=133.30, dLR= -9.37),
    "upper_leg":  dict(L=167.30, R=139.82, dLR= 27.48),
    "lower_leg":  dict(L=130.08, R=135.70, dLR= -5.62),
    "foot":       dict(L=124.33, R=149.35, dLR=-25.02),
}

GLOVE_OUTLIER_DELTAS = [
    ("L", "pinky",  "MCP", 33.2),
    ("L", "pinky",  "MCP", 30.7),
    ("L", "middle", "PIP", 36.7),
    ("L", "middle", "DIP", 33.0),
    ("L", "middle", "PIP", 43.0),
    ("L", "middle", "DIP", 38.7),
    ("L", "middle", "PIP", 41.2),
    ("L", "middle", "DIP", 37.1),
    ("L", "pinky",  "PIP", 48.1),
    ("L", "pinky",  "DIP", 43.3),
    ("R", "pinky",  "PIP", 49.1),
    ("R", "pinky",  "DIP", 44.2),
    ("L", "pinky",  "PIP", 44.1),
    ("L", "pinky",  "DIP", 39.7),
]

PELVIS_Z_STATIC = [
    (17623,  0.000,  0.036, 0.513),
    (17646,  0.006, -0.264, 0.236),
    (17662,  0.037,  0.001, 0.258),
    (17665,  0.032,  0.002, 0.259),
    (17668,  0.036,  0.002, 0.262),
    (17702,  0.000,  0.047, 0.401),
    (17704,  0.000,  0.034, 0.435),
    (17707,  0.000,  0.049, 0.485),
    (17709,  0.000,  0.031, 0.516),
    (17713,  0.000,  0.057, 0.573),
]

T_POSE_LINE_RANGE = (2481, 3503)
N_POSE_CALIB_LINE = 4404
K_POSE_CALIB_LINE = 7042
RENDER_SNAPSHOT_FIRST = 8225
STATIC_SIT_FIRST_SAMPLE = 17623
