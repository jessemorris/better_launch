Das ROS2 Launch-System ist vor allem für seine Umständlichkeit bekannt. Zwischen Dutzenden von Importen, Dicts die als Strings übergeben werden, und verwirrenden Konstrukten um selbst einfachste Python-Anweisungen zu imitieren, greift der verzweifelte Entwickler sehnsüchtig nach der Dokumentation - und findet lediglich Bruchstücke. Doch das muss nicht sein. In diesem Vortrag stelle ich better_launch vor, ein ROS2-package, welches das bestehende ROS2 Launch-System vollständig ersetzt.

--------------------------------

This talk addresses all developers using ROS2 on their systems. It introduces a new package which acts as a complete replacement for the not-so-userfriendly ROS2 launch system, bringing many benefits both in usability and on a technical level.

ROS2 has been a mixed bag so far. While the code base has seen several overhauls and improvements, this reinvention has also come with severe downgrades in terms of user friendliness and usability. One of the biggest and most exposed offenders is the launch system. On one hand, being able to write launch files in python is a nice feature which does away with many of the limitations the old ROS1 xml launch files came with. On the other hand, the current implementation tries to use python as a declarative language, forcing users to dance around unintuitive constructs as none of the values and nodes exist yet. Aside from its wordiness and significant challenge to new users, it comes with several technical issues, too:
- the execution order of actions is non-deterministic
- non-local code execution (e.g. passing code as a string for later execution)
- resists linting as many arguments have to be passed as strings, e.g. stringified dicts
- resists automated code analysis as actions don't have a unified way of exposing internals
- frequent zombie processes
- convoluted and obscure internals
- etc.

There are of course good reasons for the way the launch system has been implemented, at least on a superficial level. According to the design document, the intent is to treat launch files as "a description of what will happen" without executing anything. This is so that tools can "visualize and modify the launch description". The recently released [launchmap](https://github.com/Kodo-Robotics/launchmap) is able to do just that. However, given that users may define their own launch actions without a common way of inspecting them, it could be argued that even this use case is not well supported right now. There is also no good argument why the same use case couldn't be achieved using e.g. python's inspect module and/or a non-declarative syntax.

The issues in user friendliness have led to the emergence of several packages that simplify writing launch files, e.g. [simple_launch](https://github.com/oKermorgant/simple_launch) and [launch_generator](https://github.com/Tacha-S/launch_generator), while other packages like the very popular [generate_parameter_library](https://github.com/pickNikRobotics/generate_parameter_library) are dedicated to particular aspects of launch files. However, in the end they face the same technical issues mentioned above, as they just represent different ways of generating the launch descriptions.

In this talk I will introduce *better_launch*, a complete and self-contained replacement for the ROS2 launch system with no dependencies on the existing facilities. Getting started with *better_launch* is easy as it is well documented and comes with examples for various common use cases. It enables developers to write simple and intuitive launch files using regular pythonic syntax (see below). Crucially, it addresses the above issues as follows:

- Any actions taken in the launch files are executed immediately, allowing direct and meaningful interactions with nodes, topics and services as the launch process develops. This makes execution deterministic, and allows you to e.g. create remaps based on the topics a node publishes.

- Arguments to the launch file are "declared" in the form of function arguments. If type hints or defaults are given their values will be passed to the launch function as their respective python types. It is thus possible to branch based on launch parameters using natural if-else constructs, compare them to other values, etc. 

- Launch files written with *better_launch* are fully compatible with the ROS2 launch system. This means they can be started through `ros2 launch`, include regular ROS2 launch files, and even get included from regular ROS2 launch files. This is done by dynamically exposing a `generate_launch_description` which wraps the actual launch function in an `OpaqueFunction` action.

- The entire launch logic is contained within a single, fully documented package, examples included. Convenience functions exist for various common tasks like starting a `joint_state_publisher` or bridging Gazebo topics.

- *better_launch* comes with a replacement for `ros2 launch` called `bl`, which is both faster than its counterpart and provides additional features. For example, `bl` enables auto completion for launch parameters and uses the launch function's docstring to generate a `--help` text.

- *better_launch* was designed with user friendliness in mind and thus generates reformatted and colored terminal output by default. In addition, it provides an optional and unobtrusive terminal UI, which can be used for stopping and restarting nodes, triggering life cycle transitions, list a node's subscribed topics, dynamically adjust the logging level and more.

- Unless killed with SIGKILL, *better_launch* will not leave zombie processes behind. 

For comparison, here is the [turtlebot example](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Using-Substitutions.html) (73 lines) implemented in better launch (27 lines):
```python
#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this

@launch_this
def great_atuin(
    turtlesim_ns: str = "turtlesim1", 
    use_provided_red: bool = True, 
    new_background_r: int = 200,
):
    bl = BetterLaunch()

    with bl.group(turtlesim_ns):
        turtle_node = bl.node(
            package="turtlesim",
            executable="turtlesim_node",
            name="sim",
            params={"background_r": 120},
        )

        bl.call_service(
            topic=f"/{turtlesim_ns}/spawn",
            service_type="turtlesim/srv/Spawn",
            request_args={"x": 2.0, "y": 2.0, "theta": 0.2},
        )

        if use_provided_red:
            turtle_node.is_ros2_connected(timeout=None)
            turtle_node.set_live_params({"background_r": new_background_r})
```

We consider *better_launch* mature enough for general use in research applications. It is under active development and can be downloaded for free from https://github.com/dfki-ric/better_launch. We hope that *better_launch* will advance the state of ROS2 in a meaningful way.
