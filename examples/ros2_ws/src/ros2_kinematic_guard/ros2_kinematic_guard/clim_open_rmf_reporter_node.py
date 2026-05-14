#!/usr/bin/env python3
"""
clim_open_rmf_reporter_node.py

CLIM Open-RMF-style Reporter Node

Purpose
-------
Translate ros2_kinematic_guard / NARH Guard state into Open-RMF-style
JSON advisories without depending on unstable or not-yet-merged RMF message types.

This node is intentionally advisory-only.

It does NOT publish a real Open-RMF ~/delay message.
It does NOT decide that a robot is indefinitely delayed.
It does NOT replace the Plan Server, Plan Executor, robot adapter, or vendor driver.

It provides four structured JSON outputs:

1. /clim/command_execution_integrity
2. /clim/evidence_window
3. /clim/open_rmf/delay_advisory
4. /clim/resync_state

Plus a compact human-readable summary:

5. /clim/open_rmf/summary

Input
-----
/kinematic_guard/status    std_msgs/String JSON
/kinematic_guard/residual  std_msgs/Float64

Output
------
std_msgs/String JSON payloads.

Design
------
CLIM acts as an evidence provider.

Plan Executor asks:
    Should I report delay?

CLIM answers:
    The command-feedback chain is / is not causally aligned,
    with evidence and advisory confidence.
"""

from __future__ import annotations

import json
import math
import uuid
from collections import deque
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float64


# ============================================================
# Helpers
# ============================================================

def finite_or(x: Any, fallback: float = 0.0) -> float:
    try:
        x = float(x)
        return x if math.isfinite(x) else fallback
    except Exception:
        return fallback


def safe_get(data: Any, key: str, default: Any = None) -> Any:
    if not isinstance(data, dict):
        return default
    return data.get(key, default)


def flatten_float_dict(data: Any) -> Dict[str, float]:
    out: Dict[str, float] = {}
    if not isinstance(data, dict):
        return out

    for k, v in data.items():
        try:
            fv = float(v)
            if math.isfinite(fv):
                out[str(k)] = fv
        except Exception:
            continue

    return out


def json_safe(obj: Any) -> Any:
    if isinstance(obj, bytes):
        if len(obj) == 1:
            return int.from_bytes(obj, byteorder="little", signed=False)
        try:
            return obj.decode("utf-8")
        except Exception:
            return list(obj)

    if obj is None or isinstance(obj, (str, int, float, bool)):
        return obj

    if isinstance(obj, dict):
        return {str(k): json_safe(v) for k, v in obj.items()}

    if isinstance(obj, (list, tuple)):
        return [json_safe(v) for v in obj]

    return str(obj)


def clamp01(x: float) -> float:
    return max(0.0, min(1.0, float(x)))


# ============================================================
# Node
# ============================================================

class ClimOpenRmfReporterNode(Node):
    def __init__(self) -> None:
        super().__init__("clim_open_rmf_reporter_node")

        # --------------------------------------------------------
        # Input topics
        # --------------------------------------------------------
        self.declare_parameter("guard_status_topic", "/kinematic_guard/status")
        self.declare_parameter("guard_residual_topic", "/kinematic_guard/residual")

        # --------------------------------------------------------
        # Output topics
        # --------------------------------------------------------
        self.declare_parameter(
            "command_integrity_topic",
            "/clim/command_execution_integrity",
        )
        self.declare_parameter(
            "evidence_window_topic",
            "/clim/evidence_window",
        )
        self.declare_parameter(
            "delay_advisory_topic",
            "/clim/open_rmf/delay_advisory",
        )
        self.declare_parameter(
            "resync_state_topic",
            "/clim/resync_state",
        )
        self.declare_parameter(
            "summary_topic",
            "/clim/open_rmf/summary",
        )

        # --------------------------------------------------------
        # Identity / metadata
        # --------------------------------------------------------
        self.declare_parameter("robot_id", "demo_amr_001")
        self.declare_parameter("fleet_id", "demo_fleet")
        self.declare_parameter("plan_id", "demo_plan_001")
        self.declare_parameter("source", "CLIM")

        # --------------------------------------------------------
        # Advisory behavior
        # --------------------------------------------------------
        self.declare_parameter("advisory_only", True)

        # Do not publish actual Open-RMF messages in v0.1.
        self.declare_parameter("open_rmf_message_mode", "json_only")

        # Thresholds should match kinematic_guard_node / reporter_node.
        self.declare_parameter("yellow_threshold", 2.5)
        self.declare_parameter("red_threshold", 5.0)

        # How many clean windows are required before considering recovery stable.
        self.declare_parameter("required_clean_windows", 5)

        # If guard status stops arriving, previous state becomes stale.
        self.declare_parameter("status_timeout", 1.0)

        # Publishing rate.
        self.declare_parameter("publish_rate_hz", 5.0)

        # --------------------------------------------------------
        # Progress model
        # --------------------------------------------------------
        # Because this node does not depend on Open-RMF progress messages,
        # progressPoint is provided as a parameter or inferred from payload if present.
        self.declare_parameter("progress_point", 0.0)

        # Supported: manual, time_estimate
        self.declare_parameter("progress_point_mode", "manual")

        # Used only when progress_point_mode == time_estimate.
        self.declare_parameter("progress_rate_per_sec", 0.02)

        # For non-indefinite delay suggestions, provide a rough expected delay.
        self.declare_parameter("estimated_delay_seconds", 2.0)

        # --------------------------------------------------------
        # Read parameters
        # --------------------------------------------------------
        self.guard_status_topic = str(self.get_parameter("guard_status_topic").value)
        self.guard_residual_topic = str(self.get_parameter("guard_residual_topic").value)

        self.command_integrity_topic = str(self.get_parameter("command_integrity_topic").value)
        self.evidence_window_topic = str(self.get_parameter("evidence_window_topic").value)
        self.delay_advisory_topic = str(self.get_parameter("delay_advisory_topic").value)
        self.resync_state_topic = str(self.get_parameter("resync_state_topic").value)
        self.summary_topic = str(self.get_parameter("summary_topic").value)

        self.robot_id = str(self.get_parameter("robot_id").value)
        self.fleet_id = str(self.get_parameter("fleet_id").value)
        self.plan_id = str(self.get_parameter("plan_id").value)
        self.source_name = str(self.get_parameter("source").value)

        self.advisory_only = bool(self.get_parameter("advisory_only").value)
        self.open_rmf_message_mode = str(self.get_parameter("open_rmf_message_mode").value)

        self.yellow_threshold = float(self.get_parameter("yellow_threshold").value)
        self.red_threshold = float(self.get_parameter("red_threshold").value)
        self.required_clean_windows = int(self.get_parameter("required_clean_windows").value)
        self.status_timeout = float(self.get_parameter("status_timeout").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self.default_progress_point = float(self.get_parameter("progress_point").value)
        self.progress_point_mode = str(self.get_parameter("progress_point_mode").value)
        self.progress_rate_per_sec = float(self.get_parameter("progress_rate_per_sec").value)
        self.estimated_delay_seconds = float(self.get_parameter("estimated_delay_seconds").value)

        # --------------------------------------------------------
        # Runtime state
        # --------------------------------------------------------
        self.start_time_sec = self._now_sec()
        self.latest_status_payload: Dict[str, Any] = {}
        self.latest_status_receive_time: Optional[float] = None
        self.latest_residual: Optional[float] = None

        self.history = deque(maxlen=8)
        self.current_delay_id: Optional[str] = None
        self.event_counter = 0

        # --------------------------------------------------------
        # ROS interfaces
        # --------------------------------------------------------
        self.status_sub = self.create_subscription(
            String,
            self.guard_status_topic,
            self._status_callback,
            10,
        )

        self.residual_sub = self.create_subscription(
            Float64,
            self.guard_residual_topic,
            self._residual_callback,
            10,
        )

        self.command_integrity_pub = self.create_publisher(
            String,
            self.command_integrity_topic,
            10,
        )
        self.evidence_window_pub = self.create_publisher(
            String,
            self.evidence_window_topic,
            10,
        )
        self.delay_advisory_pub = self.create_publisher(
            String,
            self.delay_advisory_topic,
            10,
        )
        self.resync_state_pub = self.create_publisher(
            String,
            self.resync_state_topic,
            10,
        )
        self.summary_pub = self.create_publisher(
            String,
            self.summary_topic,
            10,
        )

        self.timer = self.create_timer(
            1.0 / max(self.publish_rate_hz, 0.5),
            self._publish_reports,
        )

        self.get_logger().info(
            "CLIM Open-RMF Reporter started | "
            f"status={self.guard_status_topic} -> "
            f"delay_advisory={self.delay_advisory_topic}"
        )

    # ============================================================
    # Time
    # ============================================================

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ============================================================
    # Callbacks
    # ============================================================

    def _status_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                raise ValueError("guard status JSON is not an object")

            self.latest_status_payload = payload
            self.latest_status_receive_time = self._now_sec()

            record = self._make_history_record(payload)
            self.history.append(record)

        except Exception as exc:
            self.get_logger().warn(f"Failed to parse guard status JSON: {exc}")

    def _residual_callback(self, msg: Float64) -> None:
        self.latest_residual = finite_or(msg.data, 0.0)

    # ============================================================
    # Payload extraction
    # ============================================================

    def _effective_payload(self) -> Dict[str, Any]:
        if self.latest_status_payload:
            return self.latest_status_payload

        return {
            "timestamp": self._now_sec(),
            "status": "WAITING_FOR_DATA",
            "action": "NONE",
            "r_nar": finite_or(self.latest_residual, 0.0),
            "safe_cmd": {"vx": 0.0, "wz": 0.0},
            "reasons": [],
            "components": {},
            "debug": {},
            "buffers": {},
        }

    def _make_history_record(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        components = flatten_float_dict(safe_get(payload, "components", {}))
        debug = flatten_float_dict(safe_get(payload, "debug", {}))

        return {
            "timestamp": finite_or(safe_get(payload, "timestamp", self._now_sec()), self._now_sec()),
            "receiveTime": self._now_sec(),
            "status": str(safe_get(payload, "status", "UNKNOWN")),
            "action": str(safe_get(payload, "action", "UNKNOWN")),
            "rNar": self._get_r_nar(payload),
            "reasons": safe_get(payload, "reasons", []),
            "components": components,
            "debug": debug,
        }

    def _get_r_nar(self, payload: Dict[str, Any]) -> float:
        r = safe_get(payload, "r_nar", None)
        if r is not None:
            return finite_or(r, 0.0)

        if self.latest_residual is not None:
            return finite_or(self.latest_residual, 0.0)

        return 0.0

    def _status_is_stale(self) -> bool:
        if self.latest_status_receive_time is None:
            return False
        return (self._now_sec() - self.latest_status_receive_time) > self.status_timeout

    def _clean_window_count(self, payload: Dict[str, Any]) -> int:
        count = int(safe_get(payload, "resync_good_count", 0) or 0)

        buffers = safe_get(payload, "buffers", {})
        if isinstance(buffers, dict):
            count = int(buffers.get("resync_good_count", count) or 0)

        return count

    def _progress_point(self, payload: Dict[str, Any]) -> float:
        p = safe_get(payload, "progress_point", None)
        if p is not None:
            return clamp01(finite_or(p, self.default_progress_point))

        if self.progress_point_mode == "time_estimate":
            elapsed = max(0.0, self._now_sec() - self.start_time_sec)
            return clamp01(self.default_progress_point + elapsed * self.progress_rate_per_sec)

        return clamp01(self.default_progress_point)

    # ============================================================
    # Classification
    # ============================================================

    def _latency_class(self, payload: Dict[str, Any], r_nar: float) -> str:
        status = str(safe_get(payload, "status", "UNKNOWN"))

        if status == "WAITING_FOR_DATA":
            return "UNKNOWN"

        if self._status_is_stale():
            return "CRITICAL"

        if status in {"RED_BRAKE", "RESYNCING"}:
            return "CRITICAL"

        if r_nar >= self.red_threshold:
            return "CRITICAL"

        if status in {"YELLOW_SLOWDOWN"}:
            return "DEGRADED"

        if r_nar >= self.yellow_threshold:
            return "DEGRADED"

        return "NORMAL"

    def _causal_alignment(self, latency_class: str, execution_state: str) -> str:
        if execution_state == "WAITING_FOR_DATA":
            return "UNKNOWN"

        if latency_class == "CRITICAL" or execution_state in {"RED_BRAKE", "RESYNCING"}:
            return "BROKEN"

        if latency_class == "DEGRADED" or execution_state == "YELLOW_SLOWDOWN":
            return "DEGRADED"

        return "ALIGNED"

    def _dominant_cause(self, payload: Dict[str, Any], latency_class: str) -> str:
        reasons = safe_get(payload, "reasons", [])
        if isinstance(reasons, list) and len(reasons) > 0:
            return str(reasons[0])

        components = flatten_float_dict(safe_get(payload, "components", {}))
        if components:
            key, value = max(components.items(), key=lambda item: abs(item[1]))
            if abs(value) > 0.0:
                return str(key).upper()

        status = str(safe_get(payload, "status", "UNKNOWN"))
        if status in {"RED_BRAKE", "RESYNCING"}:
            return "RESYNC_REQUIRED"

        if latency_class == "CRITICAL":
            return "EXECUTION_INTEGRITY_DEGRADED"

        if latency_class == "DEGRADED":
            return "COMMAND_FEEDBACK_DEGRADED"

        return "NONE"

    def _should_report_delay(self, latency_class: str, execution_state: str, r_nar: float) -> bool:
        if execution_state in {"RED_BRAKE", "RESYNCING"}:
            return True

        if latency_class == "CRITICAL":
            return True

        if r_nar >= self.red_threshold:
            return True

        return False

    def _indefinite_delay_candidate(
        self,
        payload: Dict[str, Any],
        latency_class: str,
        execution_state: str,
        r_nar: float,
    ) -> bool:
        """
        Advisory only.

        This does NOT decide Open-RMF indefinite_delay.
        It only says the Plan Executor may consider indefinite delay
        if its own policy agrees.
        """
        if execution_state == "RESYNCING":
            return True

        if self._status_is_stale():
            return True

        if latency_class == "CRITICAL" and r_nar >= 2.0 * max(self.red_threshold, 1e-9):
            return True

        return False

    def _delay_confidence(
        self,
        latency_class: str,
        execution_state: str,
        r_nar: float,
    ) -> float:
        if execution_state == "WAITING_FOR_DATA":
            return 0.0

        if execution_state == "RESYNCING":
            return max(0.75, clamp01(r_nar / max(self.red_threshold, 1e-9)))

        if latency_class == "CRITICAL":
            return max(0.70, clamp01(r_nar / max(self.red_threshold, 1e-9)))

        if latency_class == "DEGRADED":
            return max(0.35, clamp01(r_nar / max(self.red_threshold, 1e-9)))

        return 0.0

    def _recommended_vehicle_response(self, execution_state: str, latency_class: str) -> str:
        if execution_state == "WAITING_FOR_DATA":
            return "NONE"

        if execution_state == "RESYNCING":
            return "RESYNC_REQUIRED"

        if execution_state == "RED_BRAKE":
            return "HOLD_POSITION"

        if latency_class == "CRITICAL":
            return "HOLD_POSITION"

        if latency_class == "DEGRADED" or execution_state == "YELLOW_SLOWDOWN":
            return "CONTROLLED_DECELERATION"

        return "NONE"

    def _suggested_fleet_action(self, execution_state: str, latency_class: str) -> str:
        if execution_state == "WAITING_FOR_DATA":
            return "NONE"

        if execution_state == "RESYNCING":
            return "HOLD_NEW_ORDERS"

        if execution_state == "RED_BRAKE":
            return "AVOID_INTERSECTIONS"

        if latency_class == "CRITICAL":
            return "AVOID_INTERSECTIONS"

        if latency_class == "DEGRADED":
            return "REDUCE_ZONE_SPEED"

        return "NONE"

    def _ensure_delay_id(self, active: bool) -> Optional[str]:
        if active:
            if self.current_delay_id is None:
                self.current_delay_id = str(uuid.uuid4())
            return self.current_delay_id

        self.current_delay_id = None
        return None

    # ============================================================
    # Report builders
    # ============================================================

    def _build_reports(self) -> Dict[str, Dict[str, Any]]:
        payload = self._effective_payload()

        now = self._now_sec()
        r_nar = self._get_r_nar(payload)
        execution_state = str(safe_get(payload, "status", "UNKNOWN"))
        guard_action = str(safe_get(payload, "action", "UNKNOWN"))

        components = flatten_float_dict(safe_get(payload, "components", {}))
        debug = flatten_float_dict(safe_get(payload, "debug", {}))
        clean_count = self._clean_window_count(payload)

        latency_class = self._latency_class(payload, r_nar)
        causal_alignment = self._causal_alignment(latency_class, execution_state)
        dominant_cause = self._dominant_cause(payload, latency_class)

        progress_point = self._progress_point(payload)
        should_delay = self._should_report_delay(latency_class, execution_state, r_nar)
        indefinite_candidate = self._indefinite_delay_candidate(
            payload,
            latency_class,
            execution_state,
            r_nar,
        )
        delay_confidence = self._delay_confidence(latency_class, execution_state, r_nar)

        delay_id = self._ensure_delay_id(should_delay)

        recommended_vehicle_response = self._recommended_vehicle_response(
            execution_state,
            latency_class,
        )
        suggested_fleet_action = self._suggested_fleet_action(
            execution_state,
            latency_class,
        )

        command_execution_integrity = {
            "timestamp": now,
            "robotId": self.robot_id,
            "fleetId": self.fleet_id,
            "planId": self.plan_id,
            "commandExecutionIntegrity": {
                "residual": r_nar,
                "residualType": "kinematic_consistency",
                "latencyClass": latency_class,
                "causalAlignment": causal_alignment,
                "executionState": execution_state,
                "guardAction": guard_action,
                "dominantCause": dominant_cause,
                "recommendedVehicleResponse": recommended_vehicle_response,
                "suggestedFleetAction": suggested_fleet_action,
                "cleanWindowCount": clean_count,
                "requiredCleanWindowCount": self.required_clean_windows,
                "source": self.source_name,
            },
        }

        evidence_window = self._build_evidence_window(
            now=now,
            payload=payload,
            r_nar=r_nar,
            latency_class=latency_class,
            causal_alignment=causal_alignment,
            dominant_cause=dominant_cause,
            progress_point=progress_point,
            components=components,
            debug=debug,
        )

        resync_state = {
            "timestamp": now,
            "robotId": self.robot_id,
            "fleetId": self.fleet_id,
            "planId": self.plan_id,
            "resyncState": {
                "state": execution_state,
                "latencyClass": latency_class,
                "causalAlignment": causal_alignment,
                "cleanWindowCount": clean_count,
                "requiredCleanWindowCount": self.required_clean_windows,
                "releaseCondition": (
                    "fresh command + fresh odometry + residual below threshold "
                    "for required clean windows"
                ),
                "isReleaseReady": (
                    clean_count >= self.required_clean_windows
                    and latency_class in {"NORMAL", "UNKNOWN"}
                    and execution_state not in {"RED_BRAKE", "RESYNCING"}
                ),
                "advisoryOnly": self.advisory_only,
                "source": self.source_name,
            },
        }

        delayed_until_sec = None
        if should_delay and not indefinite_candidate:
            delayed_until_sec = now + max(0.0, self.estimated_delay_seconds)

        delay_advisory = {
            "timestamp": now,
            "robotId": self.robot_id,
            "fleetId": self.fleet_id,
            "delayAdvisory": {
                "advisoryOnly": self.advisory_only,
                "openRmfMessageMode": self.open_rmf_message_mode,
                "shouldReportDelay": should_delay,
                "delayId": delay_id,
                "planId": self.plan_id,
                "progressPoint": progress_point,
                "delayedUntilUnixSec": delayed_until_sec,
                "indefiniteDelayCandidate": indefinite_candidate,
                "delayConfidence": delay_confidence,
                "cause": {
                    "code": "EXECUTION_INTEGRITY_DEGRADED" if should_delay else "NONE",
                    "message": self._cause_message(
                        should_delay=should_delay,
                        causal_alignment=causal_alignment,
                        dominant_cause=dominant_cause,
                    ),
                    "dominantCause": dominant_cause,
                },
                "recommendedVehicleResponse": recommended_vehicle_response,
                "suggestedFleetAction": suggested_fleet_action,
                "source": self.source_name,
                "note": (
                    "This is JSON advisory telemetry only. "
                    "The Plan Executor or fleet adapter decides whether to publish "
                    "a real Open-RMF delay message."
                ),
            },
        }

        return {
            "command_execution_integrity": command_execution_integrity,
            "evidence_window": evidence_window,
            "resync_state": resync_state,
            "delay_advisory": delay_advisory,
        }

    def _build_evidence_window(
        self,
        now: float,
        payload: Dict[str, Any],
        r_nar: float,
        latency_class: str,
        causal_alignment: str,
        dominant_cause: str,
        progress_point: float,
        components: Dict[str, float],
        debug: Dict[str, float],
    ) -> Dict[str, Any]:
        if len(self.history) > 0:
            window_start = float(self.history[0].get("receiveTime", now))
            window_end = float(self.history[-1].get("receiveTime", now))
            status_sequence = [
                str(item.get("status", "UNKNOWN"))
                for item in self.history
            ]
            residual_sequence = [
                finite_or(item.get("rNar", 0.0), 0.0)
                for item in self.history
            ]
            reason_sequence = [
                item.get("reasons", [])
                for item in self.history
            ]
        else:
            window_start = now
            window_end = now
            status_sequence = [str(safe_get(payload, "status", "WAITING_FOR_DATA"))]
            residual_sequence = [r_nar]
            reason_sequence = []

        self.event_counter += 1

        return {
            "timestamp": now,
            "robotId": self.robot_id,
            "fleetId": self.fleet_id,
            "planId": self.plan_id,
            "evidenceWindow": {
                "windowId": f"clim-window-{self.event_counter}",
                "windowStartUnixSec": window_start,
                "windowEndUnixSec": window_end,
                "durationSec": max(0.0, window_end - window_start),
                "progressPoint": progress_point,
                "causalAlignment": causal_alignment,
                "latencyClass": latency_class,
                "residual": r_nar,
                "dominantCause": dominant_cause,
                "statusSequence": status_sequence,
                "residualSequence": residual_sequence,
                "reasonSequence": reason_sequence,
                "components": components,
                "debug": debug,
                "rawGuardStatusAvailable": bool(self.latest_status_payload),
                "source": self.source_name,
            },
        }

    def _cause_message(
        self,
        should_delay: bool,
        causal_alignment: str,
        dominant_cause: str,
    ) -> str:
        if not should_delay:
            return "No delay advisory. Command-feedback chain is currently acceptable."

        if causal_alignment == "BROKEN":
            return (
                "Command-feedback causal alignment is broken. "
                f"Dominant cause: {dominant_cause}."
            )

        if causal_alignment == "DEGRADED":
            return (
                "Command-feedback causal alignment is degraded. "
                f"Dominant cause: {dominant_cause}."
            )

        return f"Delay advisory generated. Dominant cause: {dominant_cause}."

    # ============================================================
    # Publishing
    # ============================================================

    def _publish_json(self, publisher, payload: Dict[str, Any]) -> None:
        msg = String()
        msg.data = json.dumps(json_safe(payload), indent=2)
        publisher.publish(msg)

    def _publish_reports(self) -> None:
        reports = self._build_reports()

        self._publish_json(
            self.command_integrity_pub,
            reports["command_execution_integrity"],
        )
        self._publish_json(
            self.evidence_window_pub,
            reports["evidence_window"],
        )
        self._publish_json(
            self.delay_advisory_pub,
            reports["delay_advisory"],
        )
        self._publish_json(
            self.resync_state_pub,
            reports["resync_state"],
        )

        cei = reports["command_execution_integrity"]["commandExecutionIntegrity"]
        da = reports["delay_advisory"]["delayAdvisory"]

        summary = String()
        summary.data = (
            f"plan={self.plan_id} "
            f"progress={da['progressPoint']:.3f} "
            f"state={cei['executionState']} "
            f"causal={cei['causalAlignment']} "
            f"latency={cei['latencyClass']} "
            f"R_NAR={cei['residual']:.3f} "
            f"delay_advisory={da['shouldReportDelay']} "
            f"indefinite_candidate={da['indefiniteDelayCandidate']} "
            f"fleet_action={cei['suggestedFleetAction']}"
        )
        self.summary_pub.publish(summary)


# ============================================================
# Main
# ============================================================

def main(args=None) -> None:
    rclpy.init(args=args)

    node = ClimOpenRmfReporterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
