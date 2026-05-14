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

## F. Theory & Implementation

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
