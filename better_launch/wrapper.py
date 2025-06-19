from typing import Callable, Literal, get_args
import os
import platform
from ast import literal_eval
import signal
import inspect
import logging
import click
import threading
from docstring_parser import parse as parse_docstring

from better_launch.launcher import (
    BetterLaunch,
    _bl_singleton_instance,
    _bl_include_args,
)
from better_launch.utils.better_logging import (
    PrettyLogFormatter,
    Colormode,
    init_logging,
)
from better_launch.utils.introspection import find_calling_frame
from better_launch.ros import logging as roslog


_is_launcher_defined = "__better_launch_this_defined"


def launch_this(
    launch_func: Callable = None,
    *,
    ui: bool = False,
    join: bool = True,
    screen_log_format: str = None,
    file_log_format: str = None,
    colormode: Colormode = "default",
    manage_foreign_nodes: bool = False,
):
    """Use this to decorate your launch function. The function will be run automatically. The function is allowed to block even when using the UI.

    **NOTE:** this decorator cannot be used more than once per module.

    Parameters
    ----------
    launch_func : Callable, optional
        Your launch function, typically using BetterLaunch to start ROS2 nodes.
    ui : bool, optional
        Whether to start the better_launch TUI. Superseded by the `BL_UI_OVERRIDE` environment variable and the `--bl_ui_override` argument.
    join : bool, optional
        If True, join the better_launch process. Has no effect when ui == True.
    screen_log_format : str, optional
        Customize how log output will be formatted when printing it to the screen. Will be overridden by the `BL_SCREEN_LOG_FORMAT_OVERRIDE` environment variable. See :py:class:`PrettyLogFormatter` for details.
    file_log_format : str, optional
        Customize how log output will be formatted when writing it to a file. Will be overridden by the `BL_FILE_LOG_FORMAT_OVERRIDE` environment variable. See :py:class:`PrettyLogFormatter` for details.
    colormode : Colormode, optional
        Decides what colors will be used for:
        * default: one color per log severity level and a single color for all message sources
        * severity: one color per log severity, don't colorize message sources
        * source: one color per message source, don't colorize log severity
        * none: don't colorize anything
        * rainbow: colorize log severity and give each message source its own color
        Superseded by the `BL_COLORMODE_OVERRIDE` environment variable and the `--bl_colormode_override` argument.
    manage_foreign_nodes : bool, optional
        If True, the TUI will also include node processes not started by this process. Has no effect if the TUI is not started.
    """

    def decoration_helper(func):
        return _launch_this_wrapper(
            func,
            ui=ui,
            join=join,
            screen_log_format=screen_log_format,
            file_log_format=file_log_format,
            colormode=colormode,
            manage_foreign_nodes=manage_foreign_nodes,
        )

    return decoration_helper if launch_func is None else decoration_helper(launch_func)


def _launch_this_wrapper(
    launch_func: Callable,
    ui: bool = False,
    join: bool = True,
    screen_log_format: str = None,
    file_log_format: str = None,
    colormode: Colormode = "default",
    manage_foreign_nodes: bool = False,
):
    # Globals of the calling module
    glob = find_calling_frame(_launch_this_wrapper).frame.f_globals

    if glob.get(_is_launcher_defined, False) and _bl_singleton_instance not in glob:
        # Allow using launch_this only once unless we got included from another file
        raise RuntimeError("Can only use one launch decorator")

    glob[_is_launcher_defined] = True

    # Get the filename of the original launchfile
    # NOTE be careful not to instantiate BetterLaunch before launch_func has run
    if not _bl_singleton_instance in glob:
        BetterLaunch._launchfile = find_calling_frame(_launch_this_wrapper).filename
        print(f"Starting launch file:\n{BetterLaunch._launchfile}\n")
        print(f"Log files will be saved at\n{roslog.launch_config.log_dir}\n")
        print("==================================================")
    else:
        # We have been included from another file, run the launch function and skip the remaining
        # initialization as its already been taken care of
        bl: BetterLaunch = glob[_bl_singleton_instance]

        includefile = find_calling_frame(_launch_this_wrapper).filename
        include_args = glob[_bl_include_args]
        bl.logger.info(f"Including launch file: {includefile} (args={include_args})")

        # Pass only those arguments that actually match the function's signature
        sig = inspect.signature(launch_func)
        matched_args = {k: include_args[k] for k in sig.parameters if k in include_args}
        bound_args = sig.bind(**matched_args)
        bound_args.apply_defaults()

        launch_func(*bound_args.args, **bound_args.kwargs)

        return

    # At this point we know that we are the main launch file

    # Signal handlers have to be installed on the main thread. Since the BetterLaunch singleton
    # could be instantiated first on a different thread we do it here where we can make stronger
    # requirements.
    if threading.current_thread() != threading.main_thread():
        raise RuntimeError("launch_this must be used on the main thread")

    sigint_count = 0

    def sigint_handler(sig, frame):
        nonlocal sigint_count
        sigint_count += 1

        # Some terminals will send SIGINT multiple times on ctrl-c, so we ignore the second one
        if sigint_count == 1:
            return

        BetterLaunch()._on_sigint(sig, frame)

    def sigterm_handler(sig, frame):
        BetterLaunch()._on_sigterm(sig, frame)

    signal.signal(signal.SIGINT, sigint_handler)
    signal.signal(signal.SIGTERM, sigterm_handler)

    if platform.system() != "Windows":
        signal.signal(signal.SIGQUIT, sigterm_handler)

    # Env overrides, will be superseded by command line args if implemented
    screen_log_format = os.environ.get("BL_SCREEN_LOG_FORMAT_OVERRIDE", screen_log_format)
    file_log_format = os.environ.get("BL_FILE_LOG_FORMAT_OVERRIDE", file_log_format)
    colormode = os.environ.get("BL_COLORMODE_OVERRIDE", colormode)
    
    env_ui = os.environ.get("BL_UI_OVERRIDE", "").lower()
    if env_ui in ("enable", "true", "1"):
        ui = True
    elif env_ui in ("disable", "false", "0"):
        ui = False

    # If we were started by ros launch (e.g. through 'ros2 launch <some-bl-launch-file>') we need
    # to expose a "generate_launch_description" method instead of running by ourselves.
    #
    # Launch files in ROS2 are run by adding an IncludeLaunchDescription action to the
    # LaunchService (both found in https://github.com/ros2/launch/). When the action is resolved,
    # it ultimately leads to get_launch_description_from_python_launch_file, which imports the file
    # and then checks for a generate_launch_description function.
    #
    # See the following links for details:
    #
    # https://github.com/ros2/launch_ros/blob/rolling/ros2launch/ros2launch/command/launch.py#L125
    # https://github.com/ros2/launch_ros/blob/rolling/ros2launch/ros2launch/api/api.py#L141
    # https://github.com/ros2/launch/blob/rolling/launch/launch/actions/include_launch_description.py#L148
    # https://github.com/ros2/launch/blob/rolling/launch/launch/launch_description_sources/python_launch_file_utilities.py#L43
    stack = inspect.stack()
    for frame_info in stack:
        frame_locals = frame_info.frame.f_locals
        if "self" not in frame_locals:
            continue

        owner = frame_locals["self"]

        if type(owner).__name__ == "IncludeLaunchDescription":
            # We were included, expose the expected method in our caller's globals and return
            print(
                f"[NOTE] Launch file {os.path.basename(BetterLaunch._launchfile)} got included from ROS2"
            )

            # TODO Maybe we shouldn't?
            init_logging(
                roslog.launch_config,
                screen_log_format,
                file_log_format,
                colormode,
            )
            _expose_ros2_launch_function(launch_func)
            return

    # If we get here we were not included by ROS2

    # Expose launch_func args through click. This enables using launch files like other
    # python files, e.g. './my_better_launchfile.py --help'
    options = []
    launch_func_sig = inspect.signature(launch_func)

    # Extract more fine-grained information from the docstring
    parsed_doc = parse_docstring(launch_func.__doc__)
    launch_func_doc = parsed_doc.short_description
    param_docstrings = {p.arg_name: p.description for p in parsed_doc.params}

    # Create CLI options for click
    for param in launch_func_sig.parameters.values():
        default = None
        if param.default is not param.empty:
            default = param.default

        ptype = None
        if default is None and param.annotation is not param.empty:
            ptype = param.annotation

        options.append(
            click.Option(
                [f"--{param.name}"],
                type=ptype,
                default=default,
                show_default=True,
                help=param_docstrings.get(param.name, None),
            )
        )

    # Additional overrides for launch arguments
    def click_ui_override(ctx: click.Context, param: click.Parameter, value: str):
        if value != "unset":
            nonlocal ui
            ui = value == "enable"
        return value

    def click_colormode_override(
        ctx: click.Context, param: click.Parameter, value: str
    ):
        if value:
            nonlocal colormode
            colormode = value
        return value

    # NOTE these should be mirrored in the bl script
    options.extend(
        [
            click.Option(
                ["--bl_ui_override"],
                type=click.types.Choice(
                    ["enable", "disable", "unset"], case_sensitive=False
                ),
                show_choices=True,
                default="unset",
                help="Override to enable/disable the terminal UI",
                expose_value=False,  # not passed to our run method
                callback=click_ui_override,
            ),
            click.Option(
                ["--bl_colormode_override"],
                type=click.types.Choice(get_args(Colormode), case_sensitive=False),
                show_choices=True,
                default=get_args(Colormode)[0],
                help="Override the logging color mode",
                expose_value=False,
                callback=click_colormode_override,
            ),
        ]
    )

    @click.pass_context
    def run(ctx: click.Context, *args, **kwargs):
        init_logging(
            roslog.launch_config, screen_log_format, file_log_format, colormode
        )

        # Wrap the launch function so we can do some preparation and cleanup tasks
        def launch_func_wrapper():
            if launch_func_kwarg is not None:
                # If the launch func defines a **kwarg we can pass all extra arguments to it, with
                # the caveat that these extra arguments need to be defined as `-[-]<key> val` tuples.
                assert (
                    len(ctx.args) % 2 == 0
                ), "extra arguments need to be '--<key> <value>' tuples"
                extra_kwargs = {}

                for (i,) in range(0, len(ctx.args), 2):
                    (key,) = ctx.args[i]
                    if not key.startswith("-"):
                        raise ValueError("Extra argument keys must start with a dash")

                    val = ctx.args[i + 1]
                    try:
                        val = literal_eval(val)
                    except:
                        # Keep val as a string
                        pass

                    extra_kwargs[key.strip("-")] = val

                kwargs[launch_func_kwarg] = extra_kwargs

            # Execute the launch function!
            launch_func(*args, **kwargs)

            # Retrieve the BetterLaunch singleton
            bl = BetterLaunch()
            if join and not ui:
                bl.spin()

        # By default BetterLaunch has access to all arguments from its launch function
        bound_args = launch_func_sig.bind(*args, **kwargs)
        bound_args.apply_defaults()
        BetterLaunch._launch_func_args = dict(bound_args.arguments)

        if ui:
            from better_launch.tui.better_tui import BetterTui

            app = BetterTui(
                launch_func_wrapper, manage_foreign_nodes=manage_foreign_nodes
            )
            app.run()
        else:
            launch_func_wrapper()

    click_cmd = click.Command(
        BetterLaunch._launchfile, callback=run, params=options, help=launch_func_doc
    )

    # The launch function should be able to define kwargs and consume unspecified arguments
    argspec = inspect.getfullargspec(launch_func)
    launch_func_kwarg = argspec[2]

    if launch_func_kwarg is not None:
        click_cmd.allow_extra_args = True
        click_cmd.ignore_unknown_options = True

    try:
        click_cmd.main()
    except SystemExit as e:
        if e.code != 0:
            raise


def _expose_ros2_launch_function(launch_func: Callable):
    """Helper function that exposes a function decorated by launch_this so that it can be included by a regular ROS2 launch file. We achieve this by generating a `generate_launch_description` function and adding it to the module globals where the launch function is defined.

    Parameters
    ----------
    launch_func : Callable
        The launch function.
    """

    def generate_launch_description():
        from launch import LaunchDescription, LaunchContext
        from launch.actions import DeclareLaunchArgument, OpaqueFunction

        ld = LaunchDescription()

        # Declare launch arguments from the function signature
        sig = inspect.signature(launch_func)
        for param in sig.parameters.values():
            default = param.default
            if default is not inspect.Parameter.empty:
                default = str(param.default)

            ld.add_action(DeclareLaunchArgument(param.name, default_value=default))

        def ros2_wrapper(context: LaunchContext, *args, **kwargs):
            # args and kwargs are only used by OpaqueFunction when using it like partial
            launch_args = {}
            for k, v in context.launch_configurations.items():
                try:
                    launch_args[k] = literal_eval(v)
                except ValueError:
                    # Probably a string
                    # NOTE this should also make passing args to ROS2 much easier
                    launch_args[k] = v

            # Call the launch function
            launch_func(**launch_args)

            # Not needed right now, but opaque functions may return additional ROS2 actions
            return

        ld.add_action(OpaqueFunction(function=ros2_wrapper))
        return ld

    # Add our generate_launch_description function to the module launch_this was called from
    launch_frame = find_calling_frame(_launch_this_wrapper)
    caller_globals = launch_frame.frame.f_globals
    caller_globals["generate_launch_description"] = generate_launch_description
