# Warehouse case study with Robo-AbU

A team of TurtleBot3 robots moves crates around a warehouse. A dispatcher
node issues tasks at arbitrary times. The robots elect who executes each
task with the round-based leader-election algorithm of Audrito et al.,
"Coordinating a Robot Swarm in a Warehouse by Aggregate Programming"
(ISoLA 2026, Alg. 1), re-expressed as Robo-AbU rules. The goal is a
like-for-like comparison with their FCPP + Gazebo + Nav2 setup.

## Setup

| Piece | Choice |
| --- | --- |
| Simulator | Gazebo Harmonic, in a ROS 2 **Jazzy** container |
| Navigation | Nav2 (Jazzy), one stack per robot, AMCL on a shared static map |
| Robots | TurtleBot3 Waffle (diff drive, 360° LDS lidar), 3 robots to start, up to 6 |
| World | AWS RoboMaker small warehouse, Harmonic port from upstream PR #27 (`Juancams:ros2-jazzy`), map `maps/005` |
| Coordination | AbU-ROS, `warehouse` branch, in a ROS 2 **Humble** container (rclgo is Humble-only) |
| Agents | abusim (simulation, one container per node), later abumon |

### The Humble/Jazzy split

The agents (Humble) never use the Nav2 action type: `NavigateToPose` has an
empty result on Humble and `error_code`/`error_msg` on Jazzy. They don't
need it. Every Nav2 function the agents use goes through types that are
identical in both distros, so the AbU-ROS vehicle talks to Nav2 directly,
with no bridge node in between (verified from a Humble container,
2026-11-05):

- **Goal:** publish `geometry_msgs/PoseStamped` on `goal_pose`; `bt_navigator`
  starts the navigation itself and preempts any current goal.
- **Status:** `action_msgs/GoalStatusArray` on
  `navigate_to_pose/_action/status`. It is published once per change, so the
  subscription must be transient-local + reliable. The array can hold several
  goals; take the most recent.
- **Cancel:** service `navigate_to_pose/_action/cancel_goal`
  (`action_msgs/srv/CancelGoal`); an empty request cancels all goals.
- **Pose:** `amcl_pose` (`geometry_msgs/PoseWithCovarianceStamped`),
  transient-local, published only when AMCL updates, so only while moving.

Humble can't decode Jazzy's discovery metadata (`rmw_dds_common` changed).
Data flows, but Humble's graph view shows unknown node names, publisher
counts of 0, and logs `sequence size exceeds remaining buffer`. The vehicle
must not depend on graph queries such as publisher counts.

With `--ipc host`, Fast DDS moves data between processes on the same
machine through shared memory. A process running as a different user than
the sim (root) discovers the topics but receives no data, because it can't
open root's segments. Containers that run as a non-root user (the agents)
must set `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`.

Fallback if cross-distro trouble grows: **Humble + Harmonic**
(`ros-humble-ros-gzharmonic` from packages.osrfoundation.org) with Nav2
Humble. The topics above stay the same.

## Navigation

- **Frames (REP-105):** a shared static `map` frame from the AWS map. Each
  robot lives in namespace `/robotN`. The diff-drive plugin publishes
  `odom -> base_footprint` and AMCL publishes `map -> odom`. Initial poses
  come from the spawn poses.
- **Nav2 per robot:** AMCL, planner (NavFn or Smac 2D), controller (Regulated
  Pure Pursuit to start), behavior server and BT navigator. The local costmap
  uses the robot's own `/scan`, so the other robots appear as obstacles.
  Sharing maps between robots (the paper's §5.2) is out of scope for now and
  is a possible later extension.
- **ROS interface of the AbU-ROS vehicle, per robot (namespace `/robotN`):**

  | Topic / service | Direction | Type | Becomes |
  | --- | --- | --- | --- |
  | `amcl_pose` | in | `geometry_msgs/PoseWithCovarianceStamped` | `pose_x`, `pose_y`, `pose_yaw` |
  | `navigate_to_pose/_action/status` | in | `action_msgs/GoalStatusArray` | `nav_status`: `idle`, `active`, `succeeded`, `failed`, `canceled` |
  | `battery_state` | in | `sensor_msgs/BatteryState` | `battery`, 0–100 (same topic a real TB3 publishes) |
  | `/clock` | in | `rosgraph_msgs/Clock` | `round = floor(sim_time / T_round)`; all robots share the sim clock, so rounds agree |
  | `fault` | in | `std_msgs/Bool` | `fault` (injected crash, from the CLI or a script) |
  | `goal_pose` | out | `geometry_msgs/PoseStamped` | from `goal_x`, `goal_y`, `goal_yaw` |
  | `navigate_to_pose/_action/cancel_goal` | out | `action_msgs/srv/CancelGoal` | from `nav_cancel` |

- **AbU attributes** are all Float, in metres and radians, so the
  Integer/Float mixing problem of the copter resources doesn't arise.
  `goal_x`, `goal_y` and `goal_yaw` form one command and are assigned in the
  same `do`. A crate is a dwell at each station, timed by the rules; there's
  no physical crate for now.

## Task model

- A task is `T_k = (L_from, L_to)`. The robot goes to `L_from`, is loaded
  (a dwell, or the crate attaches), goes to `L_to`, and is unloaded.
- Tasks live in K fixed slots (no collections in AbU); task k always uses
  slot k. The dispatcher picks a free slot and learns that a slot is free
  again from the termination broadcast.
- **Cost:** `c = ω · dist / battery_fraction`, following the paper.
  - `dist` is the straight-line distance to the current target, as in the
    paper. The Nav2 path length is an optional improvement.
  - The hysteresis factor `ω < 1` applies only to the robot already BUSY on
    the task.
  - Cost is `-∞` (a large negative constant) once the robot has picked up
    the crate, so a loaded robot is never preempted.
  - Cost is `+∞` when the robot is CRASHED or busy on another task.
- **Rules:** the task-assignment rule set worked out earlier (Announce,
  Cost, Elect, Streak, Take, Exec, Yield, Done, Crash), extended with the
  phases `to_from -> loading -> to_to -> unloading`.
- **Crash while carrying:** first version, the task is lost (logged). Later
  version: the task's target becomes the crashed robot's position, then
  `L_to`.

## Traffic control (route reservation)

Nav2 doesn't coordinate robots: two robots heading for the same point jam
each other. Conflicts are resolved by rules, ATC-style (decided 2026-11-06):

- **Zones.** The warehouse is split into named zones (each shelf aisle, the
  main corridors, each room, each station as its own zone), defined with
  their adjacency in `config/zones.yaml`. A set of zones is an Integer
  bitmask (bit k = zone k); the rule language has bitwise `&` and `|`.
- **Route.** Two rule builtins in AbU-ROS (package `zones`, graph loaded
  from the file named by `ABU_ZONES_FILE`):
  - `ZoneOf(x, y)`: the bitmask bit of the zone containing a point. A robot
    always holds its current zone, including while it waits at a station.
  - `ZoneRoute(x0, y0, x1, y1)`: the zones on the shortest zone path between
    two points (Dijkstra over zone centres), as a bitmask.

  The route is known before the robot moves. Whether Nav2's actual path stays
  inside the reserved zones is to be measured against ground truth; driving
  via zone waypoints is the fallback. (Nav2's planner action can't be called
  from Humble: `ComputePathToPose` gained `error_code`/`error_msg` in Jazzy,
  like `NavigateToPose`.)
- **Zones of this warehouse** (`config/zones.yaml`, 14 zones, 24 edges):
  - W1–W4: west strip (home, D1, D2).
  - C1–C5: central hall (D3, D4).
  - A1–A5: shelf block, one band per pick station.

  The robots fit under the shelves (only the legs block them), so the bands
  connect through the shelf rows. `tools/zones_edges.py` computes the edges
  from the map (a shared boundary with ≥ 0.7 m of free passage); rerun it
  after changing a zone.
- **Whole-route reservation.** Before moving, a robot reserves every zone on
  its route, including the destination station's zone, and releases them on
  arrival or when it stops (yield, crash).
- **Protocol:**
  1. Request: a `for all` rule broadcasts `route_req` (bitmask + request id).
  2. Deny: every robot whose reserved route overlaps the request
     (`(route & ext.req) != 0`), or whose own pending request overlaps it and
     has priority (lower `agentId`), answers with a deny aimed at the
     requester's `agentId`.
  3. Grant: no deny within a short window means the route is reserved and
     the robot drives.
  4. Release: on arrival, a broadcast `route_release` wakes up denied robots,
     which request again.
- Progressive (block-by-block) reservation and time windows are possible
  later refinements.

## Steps

Each step ends with a check that must pass before the next one starts.

1. **Simulation environment**
   - Jazzy + Harmonic + Nav2 + TB3 Docker image, with GPU (NVIDIA) and
     Wayland/X11 forwarding.
   - AWS warehouse ported to Harmonic, plus its map.
   - Multi-robot launch: N namespaced TB3s, each with its own Nav2 stack.
   - **Done when:** a `ros2 action send_goal` to each `/robotN` reaches the
     goal in Gazebo.
   - **Gate G1:** a Humble container on the same `ROS_DOMAIN_ID` can echo a
     Jazzy `PoseStamped` and publish one that Jazzy receives.
   - **Status (2026-11-04): done.**
     - G1 passes: Humble and Jazzy exchange `PoseStamped` both ways over
       Fast DDS on the host network.
     - 3 robots, concurrent Nav2 goals across the warehouse all SUCCEEDED
       (40–65 s each).
     - The whole simulation uses ~1.5 cores and 1.2 GB with software
       rendering (no NVIDIA container runtime installed); lidar 5 Hz, odom
       ~30 Hz, real-time factor ~1.
     - AWS map 005 was offset from the world by (0.10, 0.14) m; the origin is
       corrected in `maps/warehouse.yaml`. After the fix, 84% of lidar hits
       land within 0.05 m of a map wall.
     - gz-transport needs `GZ_IP=127.0.0.1` under `--net host` (set in the
       image).
     - Open: AMCL error against ground truth is 0.1–0.37 m, worst in the shelf
       aisles; tune in step 2. The Gazebo GUI (`gui:=True`) has not been
       tried yet.
2. **Robots**
   - Battery simulation node (publishes `battery_state`, drains with distance
     travelled).
   - Pick/drop station positions in `config/stations.yaml`, each checked to
     be reachable.
   - AMCL tuned against Gazebo ground truth.
   - Fault: `/robotN/fault` is only a topic convention; the agent reacts to it
     (step 3).
   - **Done when:** from a Humble container, using only the interface topics,
     each robot can be sent to a goal, cancelled mid-way, and its battery
     seen dropping as it drives.
   - **Status (2026-11-05): done.**
     - The done-check passes from Humble, as a non-root user over UDP (the
       agents' setup): goal via `goal_pose`, status, `amcl_pose`, cancel
       mid-way, battery drop while driving, `fault` topic.
     - **Floor fix (the main finding).** The AWS floor's collision was a mesh
       with anisotropic friction (mu 100 / mu2 50). Under dartsim the
       TB3's wheels slipped on it, so wheel odometry was badly wrong (a
       commanded 360° turn gave ~200°) and AMCL lost the robots. Replaced
       with the same box as a primitive with mu = 1 in
       `models/aws_robomaker_warehouse_GroundB_01/model.sdf`. Also: physics
       step 3 ms (as Nav2's TB3 world) and frictionless ball casters; neither
       mattered on its own.
     - **Map.** `maps/warehouse_sim.yaml` is rebuilt from the simulated world
       by `tools/map_from_sim.py`: one robot is teleported over a 1 m grid
       and its scans are ray-cast with true poses (181 scans). It matches the
       simulated lidar exactly and covers the rooms the AWS map never
       explored. Rerun it after any change to the world. The AWS map is kept
       as `maps/warehouse.yaml`.
     - **AMCL with stock parameters**, 3 robots on separate routes, 15/15 legs
       succeeded: position error mean 0.11 m, p95 0.23 m, max 0.68 m
       (`tools/eval_amcl.py`, truth from `/robotN/ground_truth`). No tuning
       needed; `config/nav2_params.yaml` is our copy, unchanged.
     - **Stations** in `config/stations.yaml`: 5 pick (shelf aisles), 4 drop,
       1 home; all reachable, planned path 1.00–1.25× the straight line.
     - **Battery:** `scripts/battery_sim.py`, 200 m per charge (launch arg
       `battery_range_m`), `percentage` in [0, 1] as the message defines it.
       Whether a real TB3 publishes 0–1 or 0–100 is unverified, so the
       vehicle should accept both.
     - **Robot model:** `urdf/waffle.sdf.xacro` (stock Waffle without the
       depth camera, plus a true-pose publisher).
     - **Open.** Two robots sent to the same point at the same time jam each
       other (Nav2 has no multi-robot coordination), and the collision slips
       the wheels and leaves odometry with a heading error. Handled in step 3
       by the traffic control rules. Raw odometry still drifts several metres
       per route even without jams; AMCL absorbs it.
     - Not an issue: Humble's broken graph view of Jazzy nodes (the agents
       only need graph information about each other, all Humble), and the
       battery percentage scale.
   - **Navigation fixes (2026-11-09)**, after a robot got stuck in the first
     AbU run:
     - **Obstacles the lidar can't see.** Several AWS models have collision
       geometry that differs from their visuals, and the simulated lidar sees
       only visuals. The worst case: the west-wall shelf (ShelfF) has solid
       side panels (2.1 m deep, 1.64 m tall) where the lidar sees only two
       legs, so the west strip is closed bays, not open floor. Also a solid
       spine in the middle of the shelf block (x ≈ 4.3–5.2) and larger
       outlines for the clutter.
       - `tools/collision_vs_map.py` slices every model's collision geometry
         at robot heights and lists what the lidar map lacks.
       - With `--write`, it writes `maps/warehouse_nav` = lidar map + those
         outlines. Nav2, `zones_edges.py` and the eval's clearance metric use
         it; `maps/warehouse_sim` stays the lidar-only map.
     - **Spawn height.** Robots spawned at z = 0.01, inside the floor box
       (surface 0.034), so their wheels were embedded and slipped (a 360°
       turn lost ~20°). Now z = 0.05. `map_from_sim.py` was changed the same
       way.
     - **Nav2 parameters** (`config/nav2_params.yaml`):
       - local costmap: the real Waffle footprint polygon (the stock circle
         r = 0.22 missed the rear corners), inflation 0.6;
       - global costmap: robot_radius 0.26, inflation 0.8;
       - DWB BaseObstacle.scale 0.02 → 0.05;
       - collision monitor footprint topic made relative (the stock
         multi-robot value was absolute, so the monitor never had a
         footprint).
     - **Result:** robot1's west-strip route alone, 4/4 legs ok, closest
       approach to an obstacle 0.74 m (was 0.21 m and stuck), AMCL error
       max 0.19 m. With three robots and no traffic control, the remaining
       failure is robots meeting head-on, which is the AbU traffic layer's
       job.
     - **Speed and controller (2026-11-09).** Max speed 0.26 → 0.45 m/s and
       turning 1.0 → 1.5 rad/s. The simulated drive allows 0.46 m/s and
       1.9 rad/s; a real TB3 Waffle is limited to 0.26 m/s. Acceleration
       limits in Nav2 now match the drive (1 m/s², 2 rad/s²). At 0.45 m/s,
       DWB wedged a robot on a shelf post at an aisle mouth (about 0.9 m
       wide), so the controller is now Regulated Pure Pursuit:
       - 0.45 m/s in the open;
       - slows near obstacles (cost-regulated) and in tight turns, down to
         0.15 m/s;
       - stops when a collision is less than 1 s ahead.

       Result: the three-task AbU scenario ran 3 rounds in a row, all tasks
       done in 190 / 186 / 153 s, with no Nav2 progress failures or aborted
       goals.
3. **AbU-ROS (`warehouse` branch)**
   - A `Nav2Robot` vehicle and resource using the interface above directly
     (rclgo, Humble bindings). Written fresh; the `turtlebot` branch is only a
     reference, not merged.
   - Zone graph (`config/zones.yaml`) and the `ZoneOf`/`ZoneRoute` builtins;
     traffic control rules (see "Traffic control").
   - **Status (2026-11-06):**
     - Done: `vehicles/nav2Robot.go` + `rosresources/Nav2RobotResource.go`,
       tested end to end against the sim (goal via `goal_seq`, cancel via
       `cancel_seq`, inputs pose/nav_status/battery/round).
     - Done: package `zones` + builtins, unit-tested, and checked inside AbU
       rules (bitwise `&` on masks works in conditions).
     - Note: `nav_status` is the status of Nav2's latest goal, whoever sent
       it (at start-up it can still read `succeeded` from an old goal). The
       rules must only trust it after a goal they sent themselves.
     - Next: task-assignment + traffic-control rules, dispatcher.
   - Agent containers that run as a non-root user set
     `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`.
   - Rule set with phases; dispatcher node.
   - YAML loader extension so one rule can hold several `for` and `for all`
     tasks.
   - **Done when:** in a Go test program, 3 robots and a dispatcher complete
     a stream of tasks; killing the busy robot makes another take over.
4. **abusim / abumon agents**
   - New node type, scenario YAML, logging.
   - Benchmarks comparable to the paper: total walk, active processes, and
     rounds per task, with and without opportunistic preemption, against
     random assignment.

## Workspace layout (planned)

```
warehouse_rabu/
  docker/            Dockerfiles (sim: Jazzy; agents: Humble), compose file
  ros2_ws/src/
    warehouse_sim/   launch files, world, map, Nav2 params, spawn poses,
                     stations, battery node, robot model
  tools/             eval_amcl.py (AMCL vs truth), map_from_sim.py (map from
                     the simulated world)
  rules/             Robo-AbU rule sets
  scenarios/         abusim scenarios
  PLAN.md
```

## Risks

- Humble/Jazzy DDS interoperability isn't officially supported; gate G1
  decides early.
- The AWS world is heavy. A third-party project switched to a lighter
  custom world because of it. With 4–6 TB3s we may need to strip meshes or
  reduce lidar rates.
- PR #27 is unmerged and the upstream repository was archived in July 2026,
  so we vendor the ported world into `warehouse_sim`.
- The `turtlebot` branch vehicle is a stub (goROSetta movement messages, no
  inputs). The Nav2 vehicle is new code, not a port.
