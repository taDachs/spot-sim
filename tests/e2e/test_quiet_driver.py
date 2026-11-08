"""Slice 7: a quiet driver.

Runs last (see conftest): idling and a stand / drive / sit cycle must not make
the driver log warnings or errors, on-demand services must answer cleanly,
and nothing the driver did during the whole suite may have hit an
unimplemented Spot API method (the sim lists those in a shared report file).
"""

import os
import time
import typing

from rcl_interfaces.msg import Log
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from spot_msgs.srv import Dock, ListWorldObjects, MutateWorldObject

from conftest import SPOT_NAME, RosHarness, assert_success
from driving import (
    Drive,
    claim_power_on_and_stand,
    power_off_and_release,
    twist,
    wait_for_body_height,
)

QUIET_IDLE_SEC = float(os.environ.get("QUIET_IDLE_SEC", "60"))
REPORT_FILE = "/reports/unimplemented_api_calls.txt"
SIT_HEIGHT = 0.20


def test_idle_and_drive_cycle_are_quiet(ros: RosHarness) -> None:
    complaints: typing.List[str] = []

    def on_log(msg: Log) -> None:
        if msg.level >= Log.WARN and msg.name.startswith(f"{SPOT_NAME}."):
            complaints.append(f"[{msg.name}] {msg.msg}")

    qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
    sub = ros.node.create_subscription(Log, "/rosout", on_log, qos)
    try:
        end = time.monotonic() + QUIET_IDLE_SEC
        while time.monotonic() < end:
            ros.executor.spin_once(timeout_sec=0.1)
        claim_power_on_and_stand(ros)
        drive = Drive(ros)
        try:
            drive.drive_for(twist(v_x=0.4, v_rot=0.3), sim_seconds=2.0)
            drive.wait_until_still()
        finally:
            drive.close()
        power_off_and_release(ros)
        end = time.monotonic() + 3.0
        while time.monotonic() < end:
            ros.executor.spin_once(timeout_sec=0.1)
    finally:
        ros.node.destroy_subscription(sub)
    assert not complaints, "driver warnings/errors:\n" + "\n".join(complaints)


def test_on_demand_services_answer_cleanly(ros: RosHarness) -> None:
    listed = ros.call(ListWorldObjects, "list_world_objects", ListWorldObjects.Request())
    assert listed.response.header.error.code.value in (0, 1)  # unset or CODE_OK
    # An empty mutation is answered (rejected as invalid), not left unimplemented.
    ros.call(MutateWorldObject, "mutate_world_objects", MutateWorldObject.Request())

    claim_power_on_and_stand(ros)
    dock = ros.call(Dock, "dock", Dock.Request(dock_id=520))
    assert not dock.success, "there is no dock in the simulated world"
    undock = ros.trigger("undock")
    assert not undock.success
    assert_success(ros.trigger("self_right"))
    wait_for_body_height(ros, SIT_HEIGHT, tolerance=0.04)
    power_off_and_release(ros)


def test_no_unimplemented_api_calls() -> None:
    assert os.path.exists(REPORT_FILE), "sim report missing (compose volume 'reports')"
    with open(REPORT_FILE, encoding="utf-8") as f:
        calls = [line.strip() for line in f if line.strip()]
    assert not calls, "the driver called unimplemented Spot API methods:\n" + "\n".join(calls)
