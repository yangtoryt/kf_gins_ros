#!/usr/bin/env python3
"""
PX4 parameter auto-setter via MAVROS
启动后将预设参数写入 PX4（例如 MAV_USEHILGPS），确保注入 GNSS 生效。
"""
import time
import rclpy
from rclpy.exceptions import ParameterAlreadyDeclaredException, ParameterUninitializedException
from rclpy.node import Node

from rcl_interfaces.msg import ParameterValue, ParameterType
from mavros_msgs.msg import State
from mavros_msgs.msg import ParamValue
from mavros_msgs.srv import ParamSet, ParamSetV2


def _parse_kv_list(items, value_type, logger):
    parsed = []
    for item in items:
        if "=" not in item:
            logger.error(f"Invalid param '{item}', expected NAME=VALUE.")
            continue
        name, value = item.split("=", 1)
        name = name.strip()
        value = value.strip()
        if not name:
            logger.error(f"Invalid param '{item}', empty name.")
            continue
        try:
            if value_type == "int":
                parsed.append((name, "int", int(value)))
            elif value_type == "float":
                parsed.append((name, "float", float(value)))
            elif value_type == "bool":
                if value.lower() in ("true", "1", "yes", "on"):
                    parsed.append((name, "bool", True))
                elif value.lower() in ("false", "0", "no", "off"):
                    parsed.append((name, "bool", False))
                else:
                    raise ValueError("bool expects true/false")
        except Exception as exc:
            logger.error(f"Invalid param '{item}': {exc}")
    return parsed


class Px4ParamSetter(Node):
    def __init__(self):
        super().__init__("px4_param_setter")

        try:
            self.declare_parameter("use_sim_time", False)
        except ParameterAlreadyDeclaredException:
            pass

        self.declare_parameter("service_set_v2", "/mavros/param/set_v2")
        self.declare_parameter("service_set", "/mavros/param/set")
        self.declare_parameter("wait_timeout_sec", 3.0)
        self.declare_parameter("retry_count", 8)
        self.declare_parameter("retry_sleep_sec", 1.0)
        self.declare_parameter("force_set", True)
        self.declare_parameter("auto_detect_service", True)
        self.declare_parameter("state_topic", "/mavros/state")
        self.declare_parameter("wait_for_connection", True)
        self.declare_parameter("connection_timeout_sec", 10.0)
        self.declare_parameter("param_ints", rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter("param_floats", rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter("param_bools", rclpy.Parameter.Type.STRING_ARRAY)

        self.service_set_v2 = self.get_parameter("service_set_v2").value
        self.service_set = self.get_parameter("service_set").value
        self.wait_timeout_sec = float(self.get_parameter("wait_timeout_sec").value)
        self.retry_count = int(self.get_parameter("retry_count").value)
        self.retry_sleep_sec = float(self.get_parameter("retry_sleep_sec").value)
        self.force_set = bool(self.get_parameter("force_set").value)
        self.auto_detect_service = bool(self.get_parameter("auto_detect_service").value)
        self.state_topic = self.get_parameter("state_topic").value
        self.wait_for_connection = bool(self.get_parameter("wait_for_connection").value)
        self.connection_timeout_sec = float(self.get_parameter("connection_timeout_sec").value)

        param_ints = self._get_string_array_param("param_ints")
        param_floats = self._get_string_array_param("param_floats")
        param_bools = self._get_string_array_param("param_bools")

        self.params = []
        self.params.extend(_parse_kv_list(param_ints, "int", self.get_logger()))
        self.params.extend(_parse_kv_list(param_floats, "float", self.get_logger()))
        self.params.extend(_parse_kv_list(param_bools, "bool", self.get_logger()))

        self.client_v2 = None
        self.client = None
        self._connected = False
        self.state_sub = self.create_subscription(State, self.state_topic, self._on_state, 10)

    def _get_string_array_param(self, name):
        try:
            value = self.get_parameter(name).value
        except ParameterUninitializedException:
            return []
        if value is None:
            return []
        if isinstance(value, str):
            return [value]
        return list(value)

    def _wait_for_client(self, client, label):
        for attempt in range(self.retry_count):
            if client.wait_for_service(timeout_sec=self.wait_timeout_sec):
                return True
            self.get_logger().warn(f"Waiting for {label} ({attempt + 1}/{self.retry_count})...")
            time.sleep(self.retry_sleep_sec)
        return False

    def _on_state(self, msg: State):
        self._connected = bool(msg.connected)

    def _wait_for_connection(self):
        if not self.wait_for_connection:
            return True
        deadline = time.time() + max(1.0, self.connection_timeout_sec)
        while time.time() < deadline and not self._connected:
            rclpy.spin_once(self, timeout_sec=0.2)
        if not self._connected:
            self.get_logger().warn(
                f"MAVROS not connected on {self.state_topic} (timeout {self.connection_timeout_sec}s)."
            )
            return False
        return True

    def _find_service_by_type(self, type_name):
        for name, types in self.get_service_names_and_types():
            if type_name in types:
                return name
        return None

    def _wait_for_service_by_type(self, type_name, timeout_sec):
        deadline = time.time() + max(1.0, float(timeout_sec))
        while time.time() < deadline:
            name = self._find_service_by_type(type_name)
            if name:
                return name
            rclpy.spin_once(self, timeout_sec=0.2)
        return None

    def _ensure_v2_client(self, name):
        if self.client is not None and name == self.service_set:
            self.destroy_client(self.client)
            self.client = None
        if self.client_v2 is None or name != self.service_set_v2:
            if self.client_v2 is not None:
                self.destroy_client(self.client_v2)
            self.client_v2 = self.create_client(ParamSetV2, name)
            self.service_set_v2 = name
        return self.client_v2

    def _ensure_legacy_client(self, name):
        if self.client_v2 is not None and name == self.service_set_v2:
            self.destroy_client(self.client_v2)
            self.client_v2 = None
        if self.client is None or name != self.service_set:
            if self.client is not None:
                self.destroy_client(self.client)
            self.client = self.create_client(ParamSet, name)
            self.service_set = name
        return self.client

    def _resolve_service(self, current_name, type_name):
        use_v2 = type_name.endswith("ParamSetV2")
        if not self.auto_detect_service:
            if use_v2:
                return self._ensure_v2_client(current_name), current_name
            return self._ensure_legacy_client(current_name), current_name
        found = self._find_service_by_type(type_name)
        if not found:
            found = self._wait_for_service_by_type(type_name, self.wait_timeout_sec * self.retry_count)
        if found and found != current_name:
            self.get_logger().warn(
                f"Service {current_name} not found, auto-detected {found}."
            )
        name = found or current_name
        if use_v2:
            return self._ensure_v2_client(name), name
        return self._ensure_legacy_client(name), name

    def _make_param_value(self, kind, value):
        param = ParameterValue()
        if kind == "int":
            param.type = ParameterType.PARAMETER_INTEGER
            param.integer_value = int(value)
        elif kind == "float":
            param.type = ParameterType.PARAMETER_DOUBLE
            param.double_value = float(value)
        elif kind == "bool":
            param.type = ParameterType.PARAMETER_BOOL
            param.bool_value = bool(value)
        return param

    def _make_legacy_value(self, kind, value):
        param = ParamValue()
        if kind == "float":
            param.real = float(value)
            param.integer = 0
        else:
            param.integer = int(value)
            param.real = 0.0
        return param

    def apply_params(self):
        if not self.params:
            self.get_logger().warn("No PX4 params configured, skipping.")
            return True
        self._wait_for_connection()

        v2_name = self._wait_for_service_by_type(
            "mavros_msgs/srv/ParamSetV2", self.wait_timeout_sec
        )
        if v2_name:
            # MAVROS ROS 2 may advertise the V2 service at /mavros/param/set,
            # so choose the client by service type, not by the service name.
            if v2_name != self.service_set_v2:
                self.get_logger().warn(
                    f"Service {self.service_set_v2} not found, auto-detected {v2_name}."
                )
            self.client_v2 = self._ensure_v2_client(v2_name)
            use_v2 = self._wait_for_client(self.client_v2, self.service_set_v2)
            if not use_v2:
                self.get_logger().warn("ParamSetV2 not available, falling back to ParamSet.")
        else:
            use_v2 = False
            self.get_logger().warn("ParamSetV2 service not advertised, using legacy ParamSet.")

        if not use_v2:
            self.client = self._ensure_legacy_client(self.service_set)
            ok = self._wait_for_client(self.client, self.service_set)
            if not ok:
                self.client, self.service_set = self._resolve_service(self.service_set, "mavros_msgs/srv/ParamSet")
                ok = self._wait_for_client(self.client, self.service_set)
            if not ok:
                self.get_logger().error("ParamSet service not available, giving up.")
                return False

        all_ok = True
        for name, kind, value in self.params:
            if use_v2:
                req = ParamSetV2.Request()
                req.force_set = self.force_set
                req.param_id = name
                req.value = self._make_param_value(kind, value)
                future = self.client_v2.call_async(req)
            else:
                req = ParamSet.Request()
                req.param_id = name
                req.value = self._make_legacy_value(kind, value)
                future = self.client.call_async(req)

            rclpy.spin_until_future_complete(self, future, timeout_sec=self.wait_timeout_sec)
            result = future.result()
            if result is None or not result.success:
                self.get_logger().error(f"Failed to set PX4 param {name} using {self.service_set_v2 if use_v2 else self.service_set}.")
                if use_v2:
                    self.get_logger().warn("Retrying with legacy ParamSet.")
                    use_v2 = False
                    self.client = self._ensure_legacy_client(self.service_set)
                    if self._wait_for_client(self.client, self.service_set):
                        req = ParamSet.Request()
                        req.param_id = name
                        req.value = self._make_legacy_value(kind, value)
                        future = self.client.call_async(req)
                        rclpy.spin_until_future_complete(self, future, timeout_sec=self.wait_timeout_sec)
                        result = future.result()
                        if result is None or not result.success:
                            self.get_logger().error(f"Failed to set PX4 param {name} using legacy ParamSet.")
                            all_ok = False
                        else:
                            self.get_logger().info(f"PX4 param set (legacy): {name} = {value}")
                            continue
                all_ok = False
            else:
                self.get_logger().info(f"PX4 param set: {name} = {value}")

        return all_ok


def main():
    rclpy.init()
    node = Px4ParamSetter()
    try:
        node.apply_params()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
