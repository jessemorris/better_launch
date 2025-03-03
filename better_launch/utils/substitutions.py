from typing import Any, Literal, Callable
import os
import re
from ast import literal_eval

from rcl_interfaces.srv import GetParameters

try:
    # Jazzy
    from rclpy.parameter import get_parameter_value
except ImportError:
    # Humble
    from ros2param.api import get_parameter_value


_sentinel = object()


class SubstitutionError(ValueError):
    """Exception type that will be thrown by substitution handlers.
    """
    pass


def default_substitution_handlers(
    eval_type: Literal["full", "literal", "none"]
) -> dict[str, Callable]:
    """Returns the default substitution handlers.

    Right now the following substitution handlers will be returned: 
    * `$(find <filename> <package> <subdir>)`: return the result of :py:meth:`find`. Note that substitution arguments are always sequential (not kwargs).
    * `$(arg <name> <default>)`: return the value of an argument passed to the launch function or `<default>` if it doesn't exist. Raises KeyError if no default is provided and no default was provided.
    * `$(param <full-node-name> <param>)`: retrieves the value of the ROS parameter `<param>` from the `<full-node-name>` (i.e. namespace + node name). Raises KeyError if the node does not exist or ValueError if the node does not have the specified parameter.
    * `$(env <key> <default>)`: return the value of the environment variable `<key>` or `<default>` if it doesn't exist. Raises KeyError if no default is provided and no default was provided.
    * `$(eval <python-snippet>)`: returns the result from evaluating the provided `<python-snippet>`. Typical use cases include simple math and assembling strings. **Note** that this indeed uses python's :py:func:`eval`.

    **NOTE** this function will most likely be deprecated in the future.

    Parameters
    ----------
    eval_type : Literal["full", "literal", "none"]
        How eval substitutions should be handled: `eval`, `ast.literal_eval` or not at all.

    Returns
    -------
    dict[str, Callable]
        A dict mapping substitution keys to handler functions. Note that handler functions are allowed to throw instances of :py:class:`SubstitutionError` when invalid parameters are passed.
    """
    from better_launch import BetterLaunch

    bl = BetterLaunch.instance()

    # $(package my_ros_package my_config.yaml)
    def _find(filename: str = None, package: str = None, subdir: str = None):
        return bl.find(package, filename, subdir)

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
            raise SubstitutionError("Failed to wait for node parameter service")

        req = GetParameters.Request()
        req.names = [param]
        res = srv.call(req)

        if len(res.values) != 1:
            raise SubstitutionError(
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
            # eval was disabled
            return expr

    return {
        "find": _find,
        "arg": _arg,
        "param": _param,
        "env": _env,
        "eval": _eval,
    }


def substitute_tokens(text: str, substitution_handlers: dict) -> list[str]:
    """Parses a string and replaces all substitution strings according to the provided handler functions.

    Substitution strings are expected to follow the "ROS1 pattern": `$(key *args)`, where `key` is a substitution type and `*args` are additional arguments passed to the substitution handler.

    Note that this always finds the innermost set of brackets with no brackets in between to work on. This effectively prevents the use of brackets in our expressions. This behavior may be changed in the future to be more flexible.

    .. seealso::

        :py:meth:`default_substitution_handlers`

    Parameters
    ----------
    text : str
        _description_
    substitution_handlers : dict
        _description_

    Returns
    -------
    list[str]
        _description_

    Raises
    ------
    ValueError
        _description_
    """
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
