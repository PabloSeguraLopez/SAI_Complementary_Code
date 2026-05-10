import numpy as np

DEFAULTS = {
    # --- VFH ---
    "hist_bins": 72,
    "smooth_window": 5,
    "s_max": 18,
    "threshold": 10.0,       
    "influence_distance": 2.5,

    # --- Histogram grid ---
    "grid_cell_size": 0.10,         # fiel al paper: 10 cm
    "active_window_size": 33,       # fiel al paper: 33x33
    "cv_max": 15,                   # fiel al paper: CV en [0, 15]

    # --- Sensores ---
    "body_exclusion_radius": 0.40,
    "point_z_filter": 1.2,
    "sensor_timeout": 1.0,

    # --- Control ---
    "v_max": 0.30,
    "v_min": 0.05,
    "v_max_z": 0.25,
    "yaw_rate_max": 0.15,
    "h_m": 400.0,                    # mismo orden que el histograma nuevo
    "min_height": 1.0,

    # --- Navegación ---
    "goal_tol_xy": 0.30,
    "goal_tol_z": 0.25
}