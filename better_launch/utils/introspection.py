import inspect


def find_function_frame(func):
    """Find the most recent stack frame this function is called in.

    Parameters
    ----------
    func : _type_
        _description_

    Returns
    -------
    _type_
        _description_

    Raises
    ------
    ValueError
        _description_
    """
    for frame_info in inspect.stack():
        print(frame_info.function)
        if frame_info.frame.f_code is func.__code__:
            return frame_info

    raise ValueError(f"Could not find frame of function {func}")


def find_calling_frame(func):
    """Find the most recent stack frame the specified function is called in that is NOT in the same file as the 
    function's frame (e.g. a module importing and calling a function).

    Parameters
    ----------
    func : _type_
        _description_

    Returns
    -------
    _type_
        _description_

    Raises
    ------
    ValueError
        _description_
    """
    func_frame = None

    for frame_info in inspect.stack():
        if frame_info.frame.f_code is func.__code__:
            func_frame = frame_info

        if func_frame and func_frame.filename != frame_info.filename:
            return frame_info

    raise ValueError(f"Could not find the module calling {func}")


def get_bound_arguments(func) -> dict:
    """Retrieve the arguments that were passed to the specified function.

    Parameters
    ----------
    func : _type_
        _description_

    Returns
    -------
    dict
        _description_
    """
    frame_info = find_function_frame(func)
    sig = inspect.signature(func)
    relevant_keys = set(sig.parameters.keys())
    kwargs = {k: v for k, v in frame_info.frame.f_locals.items() if k in relevant_keys}
    bound_args = sig.bind(**kwargs)
    bound_args.apply_defaults()
    return dict(bound_args.arguments)


def find_decorated_function_args(decorator_func) -> dict:
    """Retrieve the arguments of the function that the decorator was wrapping.

    Parameters
    ----------
    decorator_func : _type_
        _description_

    Returns
    -------
    dict
        _description_

    Raises
    ------
    RuntimeError
        _description_
    """

    # Find a callable function in the decorator arguments
    decorator_args = get_bound_arguments(decorator_func)

    for val in decorator_args.values():
        if inspect.isfunction(val):
            return get_bound_arguments(val)

    raise RuntimeError("Could not determine the decorated function")
