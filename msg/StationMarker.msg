# StationMarker.msg
# Published by marker_detector_node when an ArUco marker is detected.
# Phase 1: base fields only.
# Phase 2 (portfolio): adds hmac_signature for authentication.

# Detected ArUco marker ID
int32 marker_id

# Station name resolved from station_registry.yaml
string station_name

# Robot pose at detection time: [x, y, z, qx, qy, qz, qw]
float64[7] robot_pose

# Detection confidence score [0.0 – 1.0]
float64 confidence

# ROS time of detection
builtin_interfaces/Time stamp

# ── Phase 2 only (portfolio / security extension) ──────────────────────────
# HMAC-SHA256 hex digest of (marker_id + station_name) using a pre-shared key.
# Left empty ("") in Phase 1; dispatcher ignores this field in Phase 1.
string hmac_signature
