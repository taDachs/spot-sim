#!/usr/bin/env bash
# Runs end-to-end tests against fresh stacks at several simulation speeds and
# with the driver on simulation and on wall time.
#
#   tests/run_time_matrix.sh                      # test_cmd_vel.py
#   tests/run_time_matrix.sh /tests/e2e           # all end-to-end tests
#
# Each configuration is "RTF USE_SIM_TIME". Wall time only works at RTF 1:
# the driver's command timeouts would otherwise disagree with the robot's.
set -euo pipefail
cd "$(dirname "$0")/.."

TESTS=("${@:-/tests/e2e/test_cmd_vel.py}")
CONFIGS=("1.0 true" "0.5 true" "2.0 true" "1.0 false")

# Also removes leftover `docker compose run` containers: a stray driver on the
# same ROS domain would answer the tests' service calls.
down() {
  docker compose --profile test down --timeout 5 --remove-orphans >/dev/null 2>&1 || true
  docker ps -aq --filter label=com.docker.compose.project=spot-sim | xargs -r docker rm -f >/dev/null
}
trap down EXIT

failed=()
for config in "${CONFIGS[@]}"; do
  read -r rtf use_sim_time <<<"$config"
  echo "=== RTF=$rtf USE_SIM_TIME=$use_sim_time ==="
  down
  export RTF="$rtf" USE_SIM_TIME="$use_sim_time"
  docker compose up -d --wait sim >/dev/null
  docker compose up -d driver >/dev/null
  if ! docker compose --profile test run --rm e2e python3 -m pytest -v -p no:cacheprovider "${TESTS[@]}"; then
    failed+=("RTF=$rtf USE_SIM_TIME=$use_sim_time")
  fi
done

if ((${#failed[@]})); then
  printf 'FAILED: %s\n' "${failed[@]}"
  exit 1
fi
echo "all configurations passed"
