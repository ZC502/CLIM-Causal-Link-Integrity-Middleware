# CLIM-Causal-Link-Integrity-Middleware
## For Interoperability of Heterogeneous Robot Fleets
**Powered by NARH Engine**

> **The Trust Layer for Heterogeneous Robot Fleets**

CLIM fills the gap between fleet-level mission planning and robot-side physical execution. 

While standards like **Open-RMF**, **VDA 5050**, and **MassRobotics** ensure robots can *talk* to each other, CLIM ensures they can *trust* each other's execution integrity.

## Why CLIM?
In multi-vendor environments, network jitter and controller drift often lead to "Causal Discontinuity"—where a robot reports progress that doesn't match physical reality. CLIM uses the **NARH Engine** to monitor the command-feedback chain in real-time.

ROS 2 moves the robot.

Open-RMF plans shared traffic.

VDA 5050 and MassRobotics exchange fleet states.

CLIM reports whether the command-feedback chain is still causally aligned.

### Key Capabilities for Open-RMF
* **Quantitative Delay Advisory**: Provides a mathematical trigger for the `~/delay` topic.
* **Progress Integrity Evidence**: Validates if the reported `~/progress` aligns with physical odometry.
* **Causal Evidence Window**: A "Black Box" for multi-vendor integration, providing forensic data when execution fails.


## Open-RMF alignment: Delay and Progress Integrity

Open-RMF next-generation traffic management discussions introduce the idea of a `~/delay` topic, allowing a Plan Executor to report that a robot cannot move past a certain progress point until a later time or indefinitely.

**CLIM can provide a quantitative signal for this kind of delay reporting.**

Instead of relying only on timeout or manual pause events, CLIM computes:

- command-feedback residual
- causal alignment state
- execution integrity state
- clean-window recovery count
- evidence window

These can be mapped into an Open-RMF-style Delay Advisory:

```json
{
  "delayAdvisory": {
    "progressPoint": 0.63,
    "indefiniteDelay": true,
    "cause": "EXECUTION_INTEGRITY_DEGRADED",
    "source": "CLIM"
  }
}
```
CLIM does not replace the Plan Server, Plan Executor, or robot adapter.

It acts as an evidence provider:

**Plan Executor asks: should I report delay?**

**CLIM answers: the command-feedback chain is no longer causally aligned.**

CLIM continuously evaluates whether commands, feedback, timing windows, and physical responses still belong to the same execution episode.CLIM does not certify that an action is semantically complete. It provides an evidence window showing whether the physical execution trace is consistent with the reported progress.

The goal is not to replace interoperability standards.

The goal is to make multi-vendor interoperability deterministic, observable, and debuggable under real-world network and feedback uncertainty.

CLIM's long-term vision is heterogeneous robot fleets.

CLIM v0.1 focuses on AMR / AGV command-execution integrity.

Future adapters may support robotic arms, mobile manipulators, quadrupeds, drones, OPC UA Robotics, and task-level standards.

CLIM can act as an execution-integrity evidence provider for Open-RMF-style plan execution.

## Architecture
```text
Plan Server
    ↓
Plan Executor
    ↓ command / action intent
┌──────────────────────────────┐
│            CLIM              │
│  Causal Link Integrity Layer │       ┌────────────────────────┐
│                              │       │ Cross-Vendor Evidence  │
│  - command-feedback check    │─────> │ Window (JSON Log)      │
│  - delay evidence window     │       └────────────────────────┘
│  - progress integrity check  │
│  - resync advisory           │
└───────────────┬──────────────┘
                ↓
Robot Driver / Vendor Adapter
                ↓
Odometry / State / Action Feedback
```
**Non-Intrusive by Design**: CLIM primarily operates in Observe Mode, providing high-fidelity telemetry without interfering with the robot's internal control loops, making it safe for immediate deployment in brownfield sites.

## A. Scope & Roadmap 
**Current Scope (v0.1)**: Focused on **AMR / AGV** (Differential Drive, Ackermann, Omni) command-execution integrity via ROS 2.

**On the Horizon**:
- Kinematic Adapters: Robotic Arms (ISO 10218-1), Quadrupedal/Bipedal motion integrity.
- Standard Bridges: Native VDA 5050 state-bridge, MassRobotics Health extension, OPC UA Robotics.
- Task Layer: Semantic action validation (did the gripper actually close based on torque-phase alignment?).

## B. Deployment Modes: Observe vs. Guard

CLIM offers two levels of integration to balance safety and control:
1. Observe Mode (Passive): CLIM monitors topics, logs Evidence Windows, and reports telemetry. It does not alter robot behavior. Ideal for initial integration and brownfield audits.
2. Guard Mode (Active): CLIM acts as an interceptor. It can hold or modify commands (e.g., `BRAKE_AND_RESYNC`) if the causal residual violates safety thresholds.

## C. Schema-Driven Configuration

CLIM is designed to be **Zero-Code** for standard robot types.

Users define their robot and environment via YAML schemas:
- `robot_profile`: Physical limits (max_accel, max_jerk).
- `model_type`: Kinematic constraints (diff_drive, ackermann).
- `telemetry_mapping`: Topic names and field-mapping (e.g., `/cmd_vel` to `/odom`).

## D. Open-RMF: Quantitative Delay Advisory

CLIM provides a quantitative, evidence-based trigger for the proposed Open-RMF `~/delay` topic.

Instead of arbitrary timeouts, CLIM emits a `DelayAdvisory` based on causal integrity:
- **Indefinite Delay Candidate**: Triggered when the command-feedback phase is completely lost.
- **Progress Point Integrity**: High-confidence verification that the robot is physically at the progress point reported by the fleet adapter.

## E. The Causal Evidence Window

When an anomaly occurs, CLIM dumps a **Black Box** snapshot:
- Synchronized slices of commands vs. feedback.
- Calculated Residual ($R_{NAR}$) and dominant error cause.
- Causal Alignment state (`ALIGNED` vs. `BROKEN`).
- Purpose: Eliminates "vendor-blaming" in multi-vendor sites by providing objective execution logs.

## F. Quick Start: Run the CLIM Open-RMF Demo

This demo runs a complete closed-loop AMR/AGV command-integrity test in ROS 2.

It starts:

```text
jitter_injector_node.py
  Injects bad Wi-Fi / 5G-style command timing failures.

kinematic_guard_node.py
  Computes NARH-lite residuals and detects command-feedback causal breaks.

synthetic_odom_provider.py
  Acts as a virtual robot body and publishes /odom.

reporter_node.py
  Publishes ROS diagnostics and VDA5050-style telemetry.

clim_open_rmf_reporter_node.py
  Publishes Open-RMF-style DelayAdvisory, EvidenceWindow, ResyncState, and CommandExecutionIntegrity JSON.
```

---

### 1. Open the ROS 2 workspace

```bash
cd /workspaces/CLIM-Causal-Link-Integrity-Middleware/examples/ros2_ws
```

For local use, replace the path with your cloned repository path:

```bash
cd CLIM-Causal-Link-Integrity-Middleware/examples/ros2_ws
```

---

### 2. Source ROS 2 Humble

```bash
source /opt/ros/humble/setup.bash
```

---

### 3. Build the workspace

```bash
colcon build --symlink-install
```

---

### 4. Source the workspace overlay

```bash
source install/setup.bash
```

Every new terminal must source both ROS 2 and this workspace:

```bash
cd /workspaces/CLIM-Causal-Link-Integrity-Middleware/examples/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

If you see:

```text
Package 'ros2_kinematic_guard' not found
```

it means the workspace overlay has not been sourced in the current terminal.

---

### 5. Run the Wi-Fi collapse pressure test

```bash
ros2 launch ros2_kinematic_guard start_pressure_test.launch.py profile:=wifi_collapse
```

You should see injected network faults such as:

```text
DUPLICATE_IN_BURST
BURST_RELEASE_count=16
REPLAY_STALE
DROP
DELAY_0.798s
```

And NARH Guard events such as:

```text
RED_BRAKE -> RESYNCING | R_NAR=5.477 | reasons=['CMD_DT_TOO_SMALL']
```

---

## Watch the CLIM Outputs

Open a new terminal:

```bash
cd /workspaces/CLIM-Causal-Link-Integrity-Middleware/examples/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

---

### 1. Compact Open-RMF-style summary

```bash
ros2 topic echo /clim/open_rmf/summary
```

Example:

```text
data: plan=demo_plan_001 progress=0.630 state=RESYNCING causal=BROKEN latency=CRITICAL R_NAR=5.477 delay_advisory=True indefinite_candidate=True fleet_action=HOLD_NEW_ORDERS
---
data: plan=demo_plan_001 progress=0.630 state=RECOVERED causal=ALIGNED latency=NORMAL R_NAR=0.000 delay_advisory=False indefinite_candidate=False fleet_action=NONE
---
```

This compact summary is the best first screenshot for Open-RMF discussions.

---

## Save Clean JSON Example Outputs

The CLIM reporter publishes JSON payloads as `std_msgs/String`.

If you run:

```bash
ros2 topic echo /clim/open_rmf/delay_advisory --once
```

ROS 2 will save the full message wrapper:

```text
data: "{\n  \"timestamp\": ... }"
---
```

For clean JSON files, extract only the `data` field and pretty-print it.

### Command Execution Integrity

```bash
ros2 topic echo /clim/command_execution_integrity --field data --once --full-length \
| python3 -c "import sys,json; print(json.dumps(json.loads(sys.stdin.read()), indent=2))" \
> command_execution_integrity_example.json
```

### Evidence Window

```bash
ros2 topic echo /clim/evidence_window --field data --once --full-length \
| python3 -c "import sys,json; print(json.dumps(json.loads(sys.stdin.read()), indent=2))" \
> evidence_window_example.json
```

### Open-RMF-style Delay Advisory

```bash
ros2 topic echo /clim/open_rmf/delay_advisory --field data --once --full-length \
| python3 -c "import sys,json; print(json.dumps(json.loads(sys.stdin.read()), indent=2))" \
> delay_advisory_example.json
```

### Resync State

```bash
ros2 topic echo /clim/resync_state --field data --once --full-length \
| python3 -c "import sys,json; print(json.dumps(json.loads(sys.stdin.read()), indent=2))" \
> resync_state_example.json
```

Now each file is a clean JSON document.

You can verify it with:

```bash
python3 -m json.tool delay_advisory_example.json
python3 -m json.tool evidence_window_example.json
python3 -m json.tool command_execution_integrity_example.json
python3 -m json.tool resync_state_example.json
```
### Optional: save raw ROS message output

If you want the original ROS message wrapper for debugging:

```bash
ros2 topic echo /clim/command_execution_integrity --once --full-length > command_execution_integrity_raw.txt
ros2 topic echo /clim/evidence_window --once --full-length > evidence_window_raw.txt
ros2 topic echo /clim/open_rmf/delay_advisory --once --full-length > delay_advisory_raw.txt
ros2 topic echo /clim/resync_state --once --full-length > resync_state_raw.txt
```

These files contain the `std_msgs/String` wrapper:

```text
data: "{ ... }"
---
```

## G. The “Silent Failure” Test

This test compares two cases:

```text
Case A:
  Bad Wi-Fi command stream directly drives the synthetic robot body.
  No CLIM guard.
  No DelayAdvisory.
  No EvidenceWindow.

Case B:
  The same bad command stream passes through CLIM.
  CLIM reports causal breaks, resync state, and fleet-level advisories.
```

The point is not that the robot completely stops moving in Case A.

The point is that the system has no structured way to explain whether the movement still matches the command stream.

---

### Case A: Run without CLIM

Terminal 1:

```bash
cd /workspaces/CLIM-Causal-Link-Integrity-Middleware/examples/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run ros2_kinematic_guard jitter_injector_node --ros-args \
  -p profile:=wifi_collapse \
  -p use_demo_cmd:=true \
  -p demo_raw_topic:=/cmd_vel_raw \
  -p input_topic:=/cmd_vel_raw \
  -p output_topic:=/cmd_vel_jittered \
  -p status_topic:=/jitter_injector/status
```

Terminal 2:

```bash
cd /workspaces/CLIM-Causal-Link-Integrity-Middleware/examples/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run ros2_kinematic_guard synthetic_odom_provider --ros-args \
  -p input_topic:=/cmd_vel_jittered \
  -p input_type:=twist \
  -p odom_topic:=/odom_raw \
  -p status_topic:=/synthetic_odom_raw/status \
  -p publish_tf:=false \
  -p slip_probability:=0.01
```

Terminal 3:

```bash
cd /workspaces/CLIM-Causal-Link-Integrity-Middleware/examples/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic echo /odom_raw
```

You will see odometry being published.

But there is no:

```text
/clim/open_rmf/delay_advisory
/clim/evidence_window
/clim/resync_state
/clim/command_execution_integrity
```

The robot body is moving, but the fleet layer receives no causal explanation.

This is the silent failure mode:

```text
The system still produces motion and odometry,
but it cannot explain whether the motion belongs to the current command episode.
```

Stop the three terminals with `Ctrl-C`.

---

### Case B: Run with CLIM

Now run the full CLIM pressure test:

```bash
ros2 launch ros2_kinematic_guard start_pressure_test.launch.py profile:=wifi_collapse
```

In a new terminal:

```bash
ros2 topic echo /clim/open_rmf/summary
```

Expected output:

```text
data: plan=demo_plan_001 progress=0.630 state=RESYNCING causal=BROKEN latency=CRITICAL R_NAR=5.477 delay_advisory=True indefinite_candidate=True fleet_action=HOLD_NEW_ORDERS
---
data: plan=demo_plan_001 progress=0.630 state=RECOVERED causal=ALIGNED latency=NORMAL R_NAR=0.000 delay_advisory=False indefinite_candidate=False fleet_action=NONE
---
```

Now the same kind of degraded timing is visible as:

```text
causal=BROKEN
latency=CRITICAL
delay_advisory=True
fleet_action=HOLD_NEW_ORDERS
```

This is the CLIM value proposition:

```text
Without CLIM:
  motion continues, but execution integrity is silent.

With CLIM:
  execution integrity becomes observable, explainable, and actionable.
```

## H. Theory & Implementation

The original NARH formulation was developed for discrete rigid-body simulation pipelines.

In that setting, a system state is advanced by a sequence of sub-operators:

```text
s[t+1] = Ψσ(k) ∘ ... ∘ Ψσ(1)(s[t])
```

where the execution order may depend on solver internals such as constraint partitioning, thread scheduling, batching, or projection steps.

The original discrete associator is written as:

```text
A(a,b,c;s) =
    ((Ψa ∘ Ψb) ∘ Ψc)(s)
  - (Ψa ∘ (Ψb ∘ Ψc))(s)
```

and the residual is:

```text
R[t] = || A(a,b,c;s[t]) ||
```

The important point is that NARH does **not** claim that the physical state space itself is mathematically invalid.

It measures order-dependent deviations introduced by discrete numerical or computational pipelines.

`ros2_kinematic_guard` applies the same idea to ROS 2 command-flow consistency.

Detailed mathematical derivation of the **NARH Engine**, SIPA background, and $R_{NAR}$ calculation logic are moved to:

🔗 docs/narh_engine.md
