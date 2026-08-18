"""
security/checks.py
──────────────────
Phase 1: stub implementations — both checks always return True so the
dispatcher's Phase 1 code path is unaffected.

Phase 2 (portfolio): replace these implementations with real HMAC verification
and pose-consistency checking without touching dispatcher_node.py.

Public API expected by dispatcher_node.py:
  verify_hmac(msg: StationMarker) -> bool
  check_pose(msg: StationMarker)  -> bool
"""

# ─── Phase 1 stubs (no-ops) ───────────────────────────────────────────────────

def verify_hmac(msg) -> bool:
    """
    Phase 1 stub — always passes.

    Phase 2 implementation:
      import hmac, hashlib
      payload = f"{msg.marker_id}{msg.station_name}".encode()
      key     = registry[msg.marker_id]['hmac_key'].encode()
      expected = hmac.new(key, payload, hashlib.sha256).hexdigest()
      return hmac.compare_digest(expected, msg.hmac_signature)
    """
    return True


def check_pose(msg) -> bool:
    """
    Phase 1 stub — always passes.

    Phase 2 implementation:
      import math
      expected = registry[msg.marker_id]['expected_pose']   # [x, y, z]
      tolerance = registry[msg.marker_id]['pose_tolerance_m']
      robot_pos = msg.robot_pose[:3]                        # [x, y, z]
      dist = math.sqrt(sum((a - b)**2 for a, b in zip(robot_pos, expected)))
      return dist <= tolerance
    """
    return True


# ─── Phase 2 real implementations (commented out; uncomment to activate) ──────
#
# import hmac as _hmac
# import hashlib
# import math
#
# _REGISTRY: dict = {}   # set by dispatcher_node during init
#
# def verify_hmac(msg) -> bool:
#     if msg.marker_id not in _REGISTRY:
#         return False
#     key     = _REGISTRY[msg.marker_id]['hmac_key'].encode()
#     payload = f"{msg.marker_id}{msg.station_name}".encode()
#     expected = _hmac.new(key, payload, hashlib.sha256).hexdigest()
#     return _hmac.compare_digest(expected, msg.hmac_signature)
#
# def check_pose(msg) -> bool:
#     if msg.marker_id not in _REGISTRY:
#         return False
#     expected  = _REGISTRY[msg.marker_id]['expected_pose']
#     tolerance = _REGISTRY[msg.marker_id]['pose_tolerance_m']
#     robot_pos = list(msg.robot_pose[:3])
#     dist = math.sqrt(sum((a - b)**2 for a, b in zip(robot_pos, expected)))
#     return dist <= tolerance
