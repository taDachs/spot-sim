"""Slice 2: claim, software E-Stop and motor power through the driver's services.

The tests run in order and share the robot: claim -> power on -> E-Stops -> release.
"""

from spot_msgs.msg import EStopState, EStopStateArray, LeaseArray, PowerState

from conftest import STARTUP_TIMEOUT_SEC, RosHarness

STATE_TIMEOUT_SEC = 15.0


def wait_for_motor_state(ros: RosHarness, state: int) -> None:
    ros.wait_for_message(
        PowerState,
        ros.topic("status/power_states"),
        STATE_TIMEOUT_SEC,
        predicate=lambda msg: msg.motor_power_state == state,
    )


def assert_success(response) -> None:
    assert response.success, response.message


def test_claim_takes_lease_and_registers_estop(ros: RosHarness) -> None:
    assert_success(ros.trigger("claim", timeout_sec=STARTUP_TIMEOUT_SEC))

    leases = ros.wait_for_message(
        LeaseArray,
        ros.topic("status/leases"),
        STATE_TIMEOUT_SEC,
        predicate=lambda msg: any(r.resource == "body" and r.lease_owner.client_name for r in msg.resources),
    )
    body = next(r for r in leases.resources if r.resource == "body")
    assert body.lease.sequence

    # The driver's software E-Stop shows up and is not stopping the robot.
    ros.wait_for_message(
        EStopStateArray,
        ros.topic("status/estop"),
        STATE_TIMEOUT_SEC,
        predicate=lambda msg: any(
            s.type == EStopState.TYPE_SOFTWARE and s.state == EStopState.STATE_NOT_ESTOPPED for s in msg.estop_states
        ),
    )


def test_power_on(ros: RosHarness) -> None:
    assert_success(ros.trigger("power_on"))
    wait_for_motor_state(ros, PowerState.STATE_ON)


def test_gentle_estop_cuts_power_and_release_allows_power_on(ros: RosHarness) -> None:
    assert_success(ros.trigger("estop/gentle"))
    wait_for_motor_state(ros, PowerState.STATE_OFF)

    assert_success(ros.trigger("estop/release"))
    assert_success(ros.trigger("power_on"))
    wait_for_motor_state(ros, PowerState.STATE_ON)


def test_hard_estop_cuts_power(ros: RosHarness) -> None:
    assert_success(ros.trigger("estop/hard"))
    wait_for_motor_state(ros, PowerState.STATE_OFF)
    ros.wait_for_message(
        EStopStateArray,
        ros.topic("status/estop"),
        STATE_TIMEOUT_SEC,
        predicate=lambda msg: any(
            s.type == EStopState.TYPE_SOFTWARE and s.state == EStopState.STATE_ESTOPPED for s in msg.estop_states
        ),
    )
    assert_success(ros.trigger("estop/release"))


def test_power_on_after_hard_estop_reacquires_estop(ros: RosHarness) -> None:
    # The driver's power_on re-registers its software E-Stop when its keepalive
    # is gone (spot_wrapper.SpotWrapper.power_on -> resetEStop), which the robot
    # allows while the motors are off. The robot itself refusing power-on while
    # E-Stopped is covered by spot_sim_core's unit tests.
    assert_success(ros.trigger("estop/hard"))
    wait_for_motor_state(ros, PowerState.STATE_OFF)
    assert_success(ros.trigger("power_on"))
    wait_for_motor_state(ros, PowerState.STATE_ON)
    ros.wait_for_message(
        EStopStateArray,
        ros.topic("status/estop"),
        STATE_TIMEOUT_SEC,
        predicate=lambda msg: all(
            s.state == EStopState.STATE_NOT_ESTOPPED for s in msg.estop_states if s.type == EStopState.TYPE_SOFTWARE
        ),
    )


def test_release_returns_lease(ros: RosHarness) -> None:
    assert_success(ros.trigger("release"))
    ros.wait_for_message(
        LeaseArray,
        ros.topic("status/leases"),
        STATE_TIMEOUT_SEC,
        predicate=lambda msg: all(not r.lease_owner.client_name for r in msg.resources if r.resource == "body"),
    )
