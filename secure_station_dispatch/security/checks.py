"""
security/checks.py
──────────────────
Phase 2 Security Module: HMAC authentication & pose-consistency verification.

Provides verification functions for DispatcherNode:
  verify_hmac(msg: StationMarker, registry: dict) -> bool
  check_pose(msg: StationMarker, registry: dict)  -> bool
"""

import hmac as _hmac
import hashlib
import math


def verify_hmac(msg, registry: dict | None = None) -> bool:
    """
    Verify HMAC-SHA256 signature of a StationMarker message.
    Payload signed: f"{msg.marker_id}{msg.station_name}"
    """
    if not registry or msg.marker_id not in registry:
        return False
    
    station = registry[msg.marker_id]
    key_str = station.get('hmac_key')
    if not key_str:
        return False

    key = key_str.encode('utf-8')
    payload = f"{msg.marker_id}{msg.station_name}".encode('utf-8')
    expected = _hmac.new(key, payload, hashlib.sha256).hexdigest()

    return _hmac.compare_digest(expected, msg.hmac_signature or '')


def check_pose(msg, registry: dict | None = None) -> bool:
    """
    Verify robot_pose in StationMarker message is within tolerance of
    the station's expected_pose in station_registry.yaml.
    """
    if not registry or msg.marker_id not in registry:
        return False

    station = registry[msg.marker_id]
    expected = station.get('expected_pose')        # [x, y, z]
    tolerance = station.get('pose_tolerance_m', 0.5)

    if expected is None:
        return True

    robot_pos = list(msg.robot_pose[:3])
    dist = math.sqrt(sum((a - b) ** 2 for a, b in zip(robot_pos, expected)))
    return dist <= tolerance
