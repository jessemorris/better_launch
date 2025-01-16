import re
import os
from typing import Any

from rcl_interfaces.srv import GetParameters

try:
    # Jazzy
    from rclpy.parameter import get_parameter_value
except ImportError:
    # Humble
    from ros2param.api import get_parameter_value


_sentinel = object()


def default_substitution_handlers(launcher):
    def sub_package(pkg: str, file: str, dir: str):
        return launcher.find(pkg, file, dir)

    def sub_eval(*args):
        return eval(" ".join(args), {}, launcher.all_args)

    def sub_arg(key: str, default: Any = _sentinel):
        if default != _sentinel:
            return launcher.all_args.get(key, default)
        return launcher.all_args[key]

    def sub_param(full_node_name: str, param: str):
        srv = launcher.ros_adapter.ros_node.create_client(
            GetParameters, f"{full_node_name}/get_parameters"
        )

        if not srv.wait_for_service(5.0):
            raise RuntimeError("Failed to wait for node parameter service")

        req = GetParameters.Request()
        req.names = [param]

        res = srv.call(req)

        if len(res.values) != 1:
            raise ValueError(
                f"Failed to retrieve parameter {param} from {full_node_name}"
            )

        return get_parameter_value(res.values[0]) or ""

    def sub_env(key: str, default: Any = _sentinel):
        if default != _sentinel:
            return os.environ.get(key, default)
        return os.environ[key]

    return {
        "package": sub_package,
        "eval": sub_eval,
        "arg": sub_arg,
        "param": sub_param,
        "env": sub_env,
    }


def substitute_tokens(text: str, substitution_handlers: dict) -> list[str]:
    # Always find the innermost set of brackets with no brackets in between.
    # TODO This effectively prevents the use of brackets in our expressions
    pattern = re.compile(r"\$\(([^()]+)\)")

    def substitute(m: re.Match):
        cmd, *args = m.group(1).split(" ")

        if cmd not in substitution_handlers:
            raise ValueError(f"Unknown substitution command '{cmd}'")

        return str(substitution_handlers[cmd](*args))

    while "$(" in text:
        text = pattern.sub(substitute, text)

    return text
