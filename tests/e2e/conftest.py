"""Fixtures for end-to-end tests against the running sim + driver stack (see compose.yaml)."""

import os
import time
import typing

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from std_srvs.srv import Trigger

SPOT_NAME = os.environ.get("SPOT_NAME", "spot")
USE_SIM_TIME = os.environ.get("USE_SIM_TIME", "true").lower() in ("1", "true")

# The driver needs a while to connect and authenticate after the sim is up.
STARTUP_TIMEOUT_SEC = 120.0


class RosHarness:
    def __init__(self) -> None:
        self.node = Node("spot_sim_e2e")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

    def topic(self, name: str) -> str:
        return f"/{SPOT_NAME}/{name}" if SPOT_NAME else f"/{name}"

    def wait_for_message(
        self,
        msg_type: type,
        topic: str,
        timeout_sec: float,
        predicate: typing.Callable[[typing.Any], bool] = lambda _: True,
        qos: QoSProfile = qos_profile_sensor_data,
    ) -> typing.Any:
        """Spin until a message on `topic` satisfies `predicate`; return it."""
        received: typing.List[typing.Any] = []

        def callback(msg: typing.Any) -> None:
            if not received and predicate(msg):
                received.append(msg)

        sub = self.node.create_subscription(msg_type, topic, callback, qos)
        try:
            deadline = time.monotonic() + timeout_sec
            while not received and time.monotonic() < deadline:
                self.executor.spin_once(timeout_sec=0.1)
        finally:
            self.node.destroy_subscription(sub)
        if not received:
            pytest.fail(f"no matching message on {topic} within {timeout_sec:.0f} s")
        return received[0]

    def wait_until(self, condition: typing.Callable[[], bool], timeout_sec: float, what: str) -> None:
        """Spin until `condition()` holds."""
        deadline = time.monotonic() + timeout_sec
        while not condition():
            if time.monotonic() > deadline:
                pytest.fail(f"timed out after {timeout_sec:.0f} s waiting for {what}")
            self.executor.spin_once(timeout_sec=0.05)

    def call(self, srv_type: type, name: str, request: typing.Any, timeout_sec: float = 60.0) -> typing.Any:
        """Call a service of the driver (relative name) and return its response."""
        client = self.node.create_client(srv_type, self.topic(name))
        try:
            deadline = time.monotonic() + timeout_sec
            while not client.service_is_ready():
                if time.monotonic() > deadline:
                    pytest.fail(f"service {self.topic(name)} not available")
                self.executor.spin_once(timeout_sec=0.1)
            future = client.call_async(request)
            while not future.done():
                if time.monotonic() > deadline:
                    pytest.fail(f"service {self.topic(name)} did not answer within {timeout_sec:.0f} s")
                self.executor.spin_once(timeout_sec=0.1)
            return future.result()
        finally:
            self.node.destroy_client(client)

    def trigger(self, name: str, timeout_sec: float = 60.0) -> Trigger.Response:
        """Call a std_srvs/Trigger service of the driver and return its response."""
        return self.call(Trigger, name, Trigger.Request(), timeout_sec)


@pytest.fixture(scope="session")
def ros() -> typing.Iterator[RosHarness]:
    rclpy.init()
    harness = RosHarness()
    try:
        yield harness
    finally:
        harness.node.destroy_node()
        rclpy.shutdown()


def pytest_collection_modifyitems(items) -> None:
    # The quiet-driver checks review everything the suite did: run them last.
    items.sort(key=lambda item: item.fspath.basename == "test_quiet_driver.py")
