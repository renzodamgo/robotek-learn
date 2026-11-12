#!/usr/bin/env bash
# The Obstacle round as the coach ran it last (12 nov 2026, 1:1 gears): start inside the lot, leave it with the
# hard-coded pulses, 3 laps, then the partial park and the hard-coded pulses that left the car parallel in the lot
# (car_picam_park_round.yaml: team field lot 0.31 / 0.3675 m). Any extra car.launch argument goes after it, e.g.
#
#   bash tools/obstacle-round.sh                        # exactly the coach's command
#   bash tools/obstacle-round.sh direction:=cw park_fixed_sense:=2
#
# Best rounds and what is still missing: WRO-Strategies docs/robot-real-pi/obstacle-runs/20261111/README.md.
set -eo pipefail
REPO=$(cd "$(dirname "$0")/.." && pwd)
exec bash "$REPO/tools/car-run.sh" obstacle start_lot:=true \
    strategy_params:="$REPO/install/robotek_obstacle_v3/share/robotek_obstacle_v3/config/car_picam_park_round.yaml" \
    motor_rps_per_mps:=8.0 motor_min_rps:=2 recover_back:=0.3 lot_exit_speed:=0.2 \
    lot_exit_steps:='L+0.25 R-0.3 L+0.25 R-0.3 L+0.25 R-0.3 C-0.2' "$@"
