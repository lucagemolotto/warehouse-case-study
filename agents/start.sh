#!/bin/bash
# Start the Robo-AbU agents of rules/warehouse.yaml against the running sim
# (docker/compose.yaml, service sim). One container, host network, one
# process per node; logs in agents/logs/<node>.log (AbU-ROS logger files in
# agents/logs/abu/).
#
# Send a task by appending a line to agents/logs/tasks.in, e.g. P3 -> D4:
#   echo 'new_fx = 3.75, new_fy = -3.94, new_tx = -1.0, new_ty = -3.0, pending = true' >> agents/logs/tasks.in
# Stop with: docker rm -f wh_agents
#
# The AbU-ROS logger reads its settings from the fixed path
# /home/aislab/aburos/logger/logger.config (overriding the environment), and
# AbU-ROS is mounted read-only, so agents/logger.config is mounted over it.
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
LAB=$(cd "$HERE/../.." && pwd)            # ~/Documents/AISLab
docker rm -f wh_agents >/dev/null 2>&1 || true
rm -rf "$HERE"/logs/*.log "$HERE"/logs/abu
mkdir -p "$HERE/logs/abu"
: > "$HERE/logs/tasks.in"
docker run -d --name wh_agents --net host --ipc host \
  -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42} \
  -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  -e ABU_ZONES_FILE=/warehouse/ros2_ws/src/warehouse_sim/config/zones.yaml \
  -v "$LAB/AbU-ROS:/home/aislab/aburos:ro" \
  -v "$HERE/logger.config:/home/aislab/aburos/logger/logger.config:ro" \
  -v "$LAB/goROSetta:/home/aislab/goROSetta:ro" \
  -v "$LAB/rosetta:/home/aislab/rosetta:ro" \
  -v "$HOME/go/pkg/mod:/home/aislab/go/pkg/mod:ro" \
  -v "$LAB/warehouse_rabu:/warehouse" \
  --entrypoint bash abu-ros-devcontainer-dev:latest /warehouse/agents/run_agents.sh
echo "started wh_agents; logs in $HERE/logs/"
