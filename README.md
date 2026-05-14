# CLIM-Causal-Link-Integrity-Middleware
## For Interoperability of Heterogeneous Robot Fleets
**Powered by NARH Engine**

CLIM provides the missing trust layer between robot-side execution and fleet-level planning.

ROS 2 moves the robot.

Open-RMF plans shared traffic.

VDA 5050 and MassRobotics exchange fleet states.

CLIM reports whether the command-feedback chain is still causally aligned.

## Open-RMF alignment: Delay and Progress Integrity

Open-RMF next-generation traffic management discussions introduce the idea of a `~/delay` topic, allowing a Plan Executor to report that a robot cannot move past a certain progress point until a later time or indefinitely.

CLIM can provide a quantitative signal for this kind of delay reporting.

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
