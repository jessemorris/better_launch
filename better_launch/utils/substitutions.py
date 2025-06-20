from typing import Any, Literal, Callable
import os
import yaml
from ast import literal_eval

from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterType


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

        try:
            value = yaml.safe_load(res.values[0])
        except:
            # Treat it as a string
            value = str(res.values[0])

        # Using get_value from ros2param will increase memory footprint by ~5MB
        if value.type == ParameterType.PARAMETER_BOOL:
            return value.bool_value
        elif value.type == ParameterType.PARAMETER_INTEGER:
            return value.integer_value
        elif value.type == ParameterType.PARAMETER_DOUBLE:
            return value.double_value
        elif value.type == ParameterType.PARAMETER_STRING:
            return value.string_value
        elif value.type == ParameterType.PARAMETER_BYTE_ARRAY:
            return list(value.byte_array_value)
        elif value.type == ParameterType.PARAMETER_BOOL_ARRAY:
            return list(value.bool_array_value)
        elif value.type == ParameterType.PARAMETER_INTEGER_ARRAY:
            return list(value.integer_array_value)
        elif value.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
            return list(value.double_array_value)
        elif value.type == ParameterType.PARAMETER_STRING_ARRAY:
            return list(value.string_array_value)
        elif value.type == ParameterType.PARAMETER_NOT_SET:
            return None
        
        return None

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


def _parse_substitution_syntax(s: str) -> list[list | str]:
    """Parses a string containing substitution tokens into a list of lists and strings.

    Parameters
    ----------
    s : str
        The input string to parse.

    Returns
    -------
    list[list | str]
        A list containing unchanged strings and nested lists of strings/lists. These nested lists
        will consist of the substitution key and its arguments, which again may be lists.

    Raises
    ------
    ValueError
        If the input string contains unbalanced parentheses or quotes.
    """
    def tokenize(s):
        i = 0
        n = len(s)
        while i < n:
            if s[i].isspace():
                i += 1
                continue
            elif s[i] == '$' and i + 1 < n and s[i+1] == '(':
                yield '$('
                i += 2
            elif s[i] == ')':
                yield ')'
                i += 1
            elif s[i] in '"\'':
                quote = s[i]
                i += 1
                start = i
                while i < n:
                    if s[i] == quote and s[i-1] != '\\':
                        break
                    i += 1
                else:
                    raise ValueError("Missing closing quote")
                yield s[start:i]
                i += 1  # skip closing quote
            else:
                start = i
                while i < n and not s[i].isspace() and s[i] not in '$()':
                    i += 1
                yield s[start:i]

    def parse(tokens):
        stack = []
        current = []
        is_key = False
        for tok in tokens:
            if tok == '$(':
                stack.append(current)
                current = []
                # Next token should be marked as a substitution key
                is_key = True
            elif tok == ')':
                if not stack:
                    raise ValueError("Unbalanced )")
                completed = current
                current = stack.pop()
                current.append(completed)
            else:
                if is_key:
                    tok = "$" + tok
                    is_key = False
                current.append(tok)
        if stack:
            raise ValueError("Unbalanced $(")
        return current

    return parse(tokenize(s))


def substitute_tokens(text: str, substitutions: dict[str, Callable[..., str]]) -> str:
    """Parses a string and replaces all substitution strings according to the provided handler functions.

    Substitution strings are expected to follow the "ROS1 pattern": `$(key *args)`, where `key` is a substitution type and `*args` are additional arguments passed to the substitution handler.

    .. seealso::

        :py:meth:`default_substitution_handlers`

    Parameters
    ----------
    text : str
        A string that may contain substitution tokens.
    substitutions : dict[str, Callable[..., str]]
        A dict mapping substitution tokens to handler functions.

    Returns
    -------
    str
        The input string with all substitution tokens handled.

    Raises
    ------
    ValueError
        If the input string contains unbalanced parentheses or quotes.
    """
    parsed = _parse_substitution_syntax(text)

    def delve(node: list | str):
        if isinstance(node, list):
            # Evaluate nested elements first
            evaluated = [delve(token) for token in node]
            key, *args = evaluated

            # Substitution keys will start with a $ (see parse() above)
            if key.startswith("$"):
                key = key[1:]

                if key not in substitutions:
                    raise KeyError(f"Unknown substitution key: {key}")
                
                return substitutions[key](*args)
            else:
                return " ".join(evaluated)
        else:
            return node

    return delve(parsed)
