from typing import Any

from rclpy.parameter import Parameter

try:
    # Jazzy
    from rclpy.parameter import parameter_value_to_python
except ImportError:
    # Humble
    from ros2param.api import get_value as get_value_humble

    parameter_value_to_python = lambda p: get_value_humble(parameter_value=p)


class LiveParamsMixin:
    """Mixin class to add interactions with ROS parameters for running nodes. This class must be mixed in with an object providing a `fullname` member.
    """
    def __init__(self):
        super().__init__()
        self._list_params_srv = None
        self._get_params_srv = None
        self._set_params_srv = None
        self._set_params_atomic_srv = None

    def list_live_params(self, *, timeout: float = 5.0) -> list[str]:
        """List the names of the ROS parameters this node has registered.

        This will only work if the node is up and running. Note that this is different from :py:meth:`params` in that this method will directly query the ROS node, while `params` is only used when starting the node.

        Parameters
        ----------
        timeout : float, optional
            How long to wait for the service to connect.

        Returns
        -------
        list[str]
            A list of this node's ROS parameters.

        Raises
        ------
        TimeoutError
            If the service fails to connect.
        """
        if not self._list_params_srv:
            from better_launch import BetterLaunch

            bl = BetterLaunch.instance()
            self._list_params_srv = bl.service_client(
                f"{self.fullname}/list_parameters",
                "rcl_interfaces/srv/ListParameters",
                timeout=timeout,
            )

        req = self._list_params_srv.srv_type.Request()
        res = self._list_params_srv.call(req)

        return res.result.names

    def get_live_params(self, *params: str, timeout: float = 5.0) -> dict[str, Any]:
        """Retrieves the values for the specified ROS parameters of this node. If no parameters are provided, all parameters will be listed.

        This will only work if the node is up and running. Note that this is different from :py:meth:`params` in that this method will directly query the ROS node, while `params` is only used when starting the node.

        Parameters
        ----------
        *params : str
            ROS parameter names to retrieve.
        timeout : float, optional
            How long to wait for the service to connect.

        Returns
        -------
        Any
            The names and values of the ROS parameters of this node as specified.

        Raises
        ------
        TimeoutError
            If the service fails to connect.
        """
        if not self._get_params_srv:
            from better_launch import BetterLaunch

            bl = BetterLaunch.instance()
            self._get_params_srv = bl.service_client(
                f"{self.fullname}/get_parameters",
                "rcl_interfaces/srv/GetParameters",
                timeout=timeout,
            )

        if not params:
            params = self.list_live_params(timeout=timeout)

        req = self._get_params_srv.srv_type.Request(names=params)
        res = self._get_params_srv.call(req)

        return {
            name: parameter_value_to_python(pval)
            for name, pval in zip(params, res.values)
        }

    def set_live_params(
        self, params: dict[str, Any], *, timeout: float = 5.0
    ) -> dict[str, bool]:
        """Sets the specified ROS parameters on this node. 
        
        This will only work if the node is up and running. This will also not change :py:meth:`params`, which is used for starting the node.

        Parameters
        ----------
        params : dict[str, Any]
            The parameters to update.
        timeout : float, optional
            How long to wait for the service to connect.

        Returns
        -------
        dict[str, bool]:
            A dict showing which parameter updates succeeded.

        Raises
        ------
        TimeoutError
            If the service fails to connect.
        """
        if not self._set_params_srv:
            from better_launch import BetterLaunch

            bl = BetterLaunch.instance()
            self._set_params_srv = bl.service_client(
                f"{self.fullname}/set_parameters",
                "rcl_interfaces/srv/SetParameters",
                timeout=timeout,
            )

        req = self._set_params_srv.srv_type.Request(
            parameters=[
                Parameter(key, value=val).to_parameter_msg()
                for key, val in params.items()
            ]
        )
        res = self._set_params_srv.call(req)

        return {
            param: item.successful for param, item in zip(params.keys(), res.results)
        }

    def set_live_params_atomic(
        self, params: dict[str, Any], *, timeout: float = 5.0
    ) -> bool:
        """Sets the specified ROS parameters on this node. No updates will be performed if any of the operations fail.

        This will only work if the node is up and running. This will also not change :py:meth:`params`, which is used for starting the node.

        Parameters
        ----------
        params : dict[str, Any]
            The parameters to update.
        timeout : float, optional
            How long to wait for the service to connect.

        Returns
        -------
        bool:
            True if all updates succeeded, False otherwise.

        Raises
        ------
        TimeoutError
            If the service fails to connect.
        """
        if not self._set_params_atomic_srv:
            from better_launch import BetterLaunch

            bl = BetterLaunch.instance()
            self._set_params_atomic_srv = bl.service_client(
                f"{self.fullname}/set_parameters_atomically",
                "rcl_interfaces/srv/SetParametersAtomically",
                timeout=timeout,
            )

        req = self._set_params_atomic_srv.srv_type.Request(
            parameters=[
                Parameter(key, value=val).to_parameter_msg()
                for key, val in params.items()
            ]
        )
        res = self._set_params_atomic_srv.call(req)

        return res.successful
