#!/usr/bin/env bash
# ═════════════════════════════════════════════════════════════════════════════
# run_navigation_checks.sh — ROS-free regression suite
# ═════════════════════════════════════════════════════════════════════════════
# Runs everything that can be verified without a ROS graph or a simulator:
#
#   1. world / registry synchronisation      (scripts/check_world_sync.py)
#   2. planner + perception budget           (test/test_waypoint_planner.py)
#   3. full offline tour simulation          (test/test_tour_simulation.py)
#
# These are the checks that pin the Phase 3 bay-collision fix.  Run them before
# every commit; they take about a second.
# ═════════════════════════════════════════════════════════════════════════════
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"
cd "$PKG_DIR"

RC=0
run() {
  echo
  echo "───────────────────────────────────────────────────────────────────"
  echo "  $1"
  echo "───────────────────────────────────────────────────────────────────"
  shift
  "$@" || RC=1
}

run "world / registry synchronisation" python3 scripts/check_world_sync.py
run "planner + perception regression"  python3 test/test_waypoint_planner.py
run "offline tour simulation"          python3 test/test_tour_simulation.py

echo
if [ "$RC" -eq 0 ]; then
  echo "═══ NAVIGATION CHECKS: PASS ═══"
else
  echo "═══ NAVIGATION CHECKS: FAIL ═══"
fi
exit "$RC"
