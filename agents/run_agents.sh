#!/bin/bash
# Inside the wh_agents container (see start.sh).
set -e
source /opt/ros/humble/setup.bash
source /home/aislab/aburos/aburos_msgs/install/setup.bash
source /home/aislab/goROSetta/goROSetta_msgs/install/setup.bash
cd /home/aislab/aburos
source cgo-flags.env
export GOMODCACHE=/home/aislab/go/pkg/mod GOFLAGS=-mod=mod GOPROXY=off
go build -o /tmp/scenario_runner ./cmd/scenario_runner/
LOGS=/warehouse/agents/logs
SCEN=/warehouse/rules/warehouse.yaml
WATCH_ROBOT=state,phase,busy_task,t1_lead,t2_lead,t3_lead,req_open,resp,denied,waiting,moving,route,held,nav_status,arrivals,battery
for r in robot1 robot2 robot3; do
  /tmp/scenario_runner -scenario $SCEN -node $r -watch $WATCH_ROBOT </dev/null > $LOGS/$r.log 2>&1 &
done
tail -n +1 -F $LOGS/tasks.in 2>/dev/null | /tmp/scenario_runner -scenario $SCEN -node dispatcher \
  -watch pending,t1_active,t2_active,t3_active > $LOGS/dispatcher.log 2>&1 &
wait
