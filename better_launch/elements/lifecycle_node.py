from typing import Any, Callable
import logging
from enum import IntEnum
from collections import deque

from lifecycle_msgs.msg import TransitionEvent, State, Transition
from lifecycle_msgs.srv import ChangeState as ChangeLifecycleState

from .node import Node


class LifecycleNode(Node):
    class LifecycleStage(IntEnum):
        PRISTINE = 0
        CONFIGURED = 1
        ACTIVE = 2
        FINALIZED = 3

    _stage_to_state_map = {
        LifecycleStage.PRISTINE: State.PRIMARY_STATE_UNCONFIGURED,
        LifecycleStage.CONFIGURED: State.PRIMARY_STATE_INACTIVE,
        LifecycleStage.ACTIVE: State.PRIMARY_STATE_ACTIVE,
        LifecycleStage.FINALIZED: State.PRIMARY_STATE_FINALIZED,
    }

    # See https://design.ros2.org/articles/node_lifecycle.html
    _transition_map = {
        State.PRIMARY_STATE_UNCONFIGURED: [
            (Transition.TRANSITION_CONFIGURE, State.PRIMARY_STATE_INACTIVE),
            (Transition.TRANSITION_UNCONFIGURED_SHUTDOWN, State.PRIMARY_STATE_FINALIZED),
        ],
        State.PRIMARY_STATE_INACTIVE: [
            (Transition.TRANSITION_INACTIVE_SHUTDOWN, State.PRIMARY_STATE_FINALIZED),
            (Transition.TRANSITION_CLEANUP, State.PRIMARY_STATE_UNCONFIGURED),
            (Transition.TRANSITION_ACTIVATE, State.PRIMARY_STATE_ACTIVE),
        ],
        State.PRIMARY_STATE_ACTIVE: [
            (Transition.TRANSITION_DEACTIVATE, State.PRIMARY_STATE_INACTIVE),
            (Transition.TRANSITION_ACTIVE_SHUTDOWN, State.PRIMARY_STATE_FINALIZED),
        ],
        State.PRIMARY_STATE_FINALIZED: [
            (Transition.TRANSITION_DESTROY, State.PRIMARY_STATE_UNKNOWN)
        ],
    }

    def __init__(
        self,
        package: str,
        executable: str,
        name: str,
        namespace: str,
        target_stage: int = LifecycleStage.PRISTINE,
        *,
        remaps: dict[str, str] = None,
        node_args: dict[str, Any] = None,
        cmd_args: list[str] = None,
        env: dict[str, str] = None,
        log_level: int = logging.INFO,
        output_config: (
            Node.LogSink | dict[Node.LogSource, set[Node.LogSink]]
        ) = "screen",
        reparse_logs: bool = True,
        on_exit: Callable = None,
        max_respawns: int = 0,
        respawn_delay: float = 0.0,
        use_shell: bool = False,
        emulate_tty: bool = False,
    ):
        from better_launch import BetterLaunch

        super().__init__(
            package,
            executable,
            name,
            namespace,
            node_args=node_args,
            remaps=remaps,
            cmd_args=cmd_args,
            env=env,
            log_level=log_level,
            output_config=output_config,
            reparse_logs=reparse_logs,
            on_exit=on_exit,
            max_respawns=max_respawns,
            respawn_delay=respawn_delay,
            use_shell=use_shell,
            emulate_tty=emulate_tty,
            start_immediately=True,
        )

        self._current_stage = LifecycleNode.LifecycleStage.PRISTINE
        self._current_state_id = State.PRIMARY_STATE_UNCONFIGURED

        launcher = BetterLaunch.instance()
        self._state_sub = launcher.ros_adapter.ros_node.create_subscription(
            TransitionEvent,
            f"{self.fullname}/transition_event",
            self._on_transition_event,
            10,
        )

        self._transition_client = launcher.ros_adapter.ros_node.create_client(
            ChangeLifecycleState, f"{self.fullname}/change_state"
        )
        if not self._transition_client.wait_for_service(5.0):
            raise RuntimeError("Could not connect to lifecycle transition service")

        if target_stage > LifecycleNode.LifecycleStage.PRISTINE:
            self.transition(target_stage)

    @property
    def stage(self) -> LifecycleStage:
        return self._current_stage

    @property
    def state_id(self) -> int:
        return self._current_state_id

    def _on_transition_event(self, evt: TransitionEvent) -> None:
        self._current_state_id = evt.goal_state.id
        for key, val in LifecycleNode._stage_to_state_map.items():
            if val == evt.goal_state.id:
                self._current_stage = key
                break
        else:
            self._current_stage = -1

    def _find_transition_path(start_state: int, goal_state: int) -> list[int]:
        # Queue for BFS: (current_state, path_to_state)
        queue = deque([(start_state, [])])
        visited = set()

        while queue:
            current_state, path = queue.popleft()

            if current_state == goal_state:
                return path

            if current_state in visited:
                continue

            visited.add(current_state)

            # Explore transitions from the current state
            for transition, next_state in LifecycleNode._transition_map.get(current_state, []):
                if next_state not in visited:
                    queue.append((next_state, path + [transition]))

        # No path found
        return []

    def _do_transition(self, transition_id: int) -> bool:
        req = ChangeLifecycleState.Request()
        req.transition.id = transition_id

        res = self._transition_client.call(req)
        return res.success

    def transition(self, target_stage: LifecycleStage) -> bool:
        # Figure out if and how we can get from our current state to the target state
        target_state_id = LifecycleNode._stage_to_state_map[target_stage]
        transition_path = self._find_transition_path(
            self._current_state_id, target_state_id
        )

        if not transition_path:
            raise ValueError(
                f"Could not find a valid transition sequence for {self._current_stage}->{target_stage}"
            )

        for transition in transition_path:
            if not self._do_transition(transition):
                self.logger.error(
                    f"Lifecycle transition {transition} for {self.name} towards stage {target_stage} failed"
                )
                return False

        # TODO we may not have received the transition update yet
        return True
