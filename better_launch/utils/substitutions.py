import re
import os
from typing import Any, Literal
from ast import literal_eval

from rcl_interfaces.srv import GetParameters

try:
    # Jazzy
    from rclpy.parameter import get_parameter_value
except ImportError:
    # Humble
    from ros2param.api import get_parameter_value


_sentinel = object()


def default_substitution_handlers(
    eval_type: Literal["full", "literal", "none"]
):
    from better_launch import BetterLaunch

    bl = BetterLaunch.instance()

    # $(package my_ros_package my_config.yaml)
    def _package(filename: str = None, package: str = None, subdir: str = None):
        return bl.find(filename=filename, package=package, subdir=subdir)

    # $(arg x 2.0)
    def _arg(key: str, default: Any = _sentinel):
        if default != _sentinel:
            return bl.launch_args.get(key, default)
        return bl.launch_args[key]

    # $(param /myrobot/my_node rate)
    def _param(full_node_name: str, param: str):
        srv = bl.shared_node.create_client(
            GetParameters, f"{full_node_name}/get_parameters"
        )

        if not srv.wait_for_service(5.0):
            raise KeyError("Failed to wait for node parameter service")

        req = GetParameters.Request()
        req.names = [param]
        res = srv.call(req)

        if len(res.values) != 1:
            raise ValueError(
                f"Failed to retrieve parameter {param} from {full_node_name}"
            )

        return get_parameter_value(res.values[0]) or ""

    # $(env ROS_DISTRO)
    def _env(key: str, default: Any = _sentinel):
        if default != _sentinel:
            return os.environ.get(key, default)
        return os.environ[key]

    # $(eval $(arg x) * 5)
    def _eval(*args):
        expr = " ".join(args)
        if eval_type == "full":
            return eval(expr, {}, dict(bl.launch_args))
        elif eval_type == "literal":
            return literal_eval(expr)
        else:
            raise RuntimeError("eval was not enabled for substitutions")

    return {
        "package": _package,
        "arg": _arg,
        "param": _param,
        "env": _env,
        "eval": _eval,
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
