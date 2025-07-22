import unittest
import time
from pathlib import Path
from concurrent.futures import Future

from better_launch import BetterLaunch
from better_launch.elements import Node

from launch_ros.actions import Node as RosLaunchNode
from rclpy.qos import qos_profile_parameters


class TestBetterLaunchExamples(unittest.TestCase):
    # @classmethod
    # def tearDownClass(cls):
    #     bl = BetterLaunch.instance()
    #     if bl:
    #         bl.shutdown("Tests concluded")

    def __init__(self, methodName="runTest"):
        super().__init__(methodName)

    def assertTalkerListenerRunning(self, talker: Node, listener: Node, topic: str) -> bool:
        time.sleep(2.0)

        self.assertTrue(talker.is_running)
        self.assertTrue(listener.is_running)

        self.assertIn(
            topic,
            talker.get_published_topics(),
            "Talker is not publishing on expected topic",
        )
        self.assertIn(
            topic,
            listener.get_subscribed_topics(),
            "Listener is not subscribed on expected topic",
        )

        talker.shutdown("Test successful", timeout=None)
        self.assertFalse(talker.is_running, "Talker failed to shutdown")

        listener.shutdown("Test successful", timeout=None)
        self.assertFalse(listener.is_running, "Listener failed to shutdown")

    def test_bl_init(self):
        bl = BetterLaunch()
        self.assertIsNotNone(bl, "BetterLaunch initialized")

        this_file = bl.find(filename=Path(__file__).name, subdir="../**")
        self.assertEqual(
            Path(bl.launchfile), Path(this_file), "Could not locate test file"
        )

        self.assertGreaterEqual(
            bl.shared_node.count_publishers("rosout"), 1, "Shared node not working"
        )

    def test_topic(self):
        bl = BetterLaunch()
        message_future = Future()

        def on_msg(msg):
            message_future.set_result(msg)

        msg_type = bl.get_ros_message_type("std_msgs/msg/String")
        sub = bl.subscriber("/test/better_launch/topic_test", msg_type, callback=on_msg, qos_profile=qos_profile_parameters)
        
        time.sleep(1.0)
        
        bl.publish_message(
            "/test/better_launch/topic_test", msg_type, {"data": "hello world"}, qos_profile=qos_profile_parameters
        )

        result = message_future.result(2.0).data
        self.assertEqual(
            result, "hello world", "Failed to receive message"
        )
        sub.destroy()

    def test_node(self):
        bl = BetterLaunch()

        talker = bl.node(
            "examples_rclpy_minimal_publisher",
            "publisher_local_function",
            "my_talker_node",
            remaps={"/topic": "/test/better_launch/chatter_node"},
        )
        listener = bl.node(
            "examples_rclpy_minimal_subscriber",
            "subscriber_member_function",
            "my_listener_node",
            remaps={"/topic": "/test/better_launch/chatter_node"},
        )

        talker.start()
        listener.start()
        self.assertTalkerListenerRunning(talker, listener, "/test/better_launch/chatter_node")

    def test_compose(self):
        bl = BetterLaunch()

        with bl.compose("my_composer"):
            talker = bl.component(
                "composition",
                "composition::Talker",
                "my_talker_comp",
                remaps={"/chatter": "/test/better_launch/chatter_comp"},
            )

        # bl.compose returns a Composer that we could reuse, but even without it's possible to reuse an
        # already running composer node (even if it wasn't started with better_launch!)
        with bl.compose("my_composer", reuse_existing=True) as composer:
            listener = bl.component(
                package="composition",
                plugin="composition::Listener",
                name="my_listener_comp",
                remaps={"/chatter": "/test/better_launch/chatter_comp"},
            )

        self.assertTalkerListenerRunning(talker, listener, "/test/better_launch/chatter_comp")

        composer.shutdown("Test successful", timeout=None)
        self.assertFalse(composer.is_running, "Composer failed to shutdown")

    def test_include(self):
        bl = BetterLaunch()

        bl.include("better_launch", "05_launch_arguments.launch.py", enable=True)

        talker = bl.query_node("/my_talker")
        self.assertIsNotNone(talker)

        listener = bl.query_node("/my_listener")
        self.assertIsNotNone(listener)

        # Included example doesn't use remaps
        self.assertTalkerListenerRunning(talker, listener, "/topic")

    def test_ros2_actions(self):
        bl = BetterLaunch()

        ros2 = bl.ros2_actions(
            RosLaunchNode(
                package="examples_rclpy_minimal_publisher",
                executable="publisher_local_function",
                name="my_talker_ros2",
                remappings=[("topic", "/test/better_launch/chatter_ros2")],
            ),
            RosLaunchNode(
                package="examples_rclpy_minimal_subscriber",
                executable="subscriber_member_function",
                name="my_listener_ros2",
                remappings=[("topic", "/test/better_launch/chatter_ros2")],
            ),
        )

        time.sleep(5.0)

        publishers = bl.shared_node.get_publishers_info_by_topic(
            "/test/better_launch/chatter_ros2"
        )
        self.assertIn(
            "my_talker_ros2",
            [p.node_name for p in publishers],
            "Talker is not publishing on expected topic",
        )

        subscribers = bl.shared_node.get_subscriptions_info_by_topic(
            "/test/better_launch/chatter_ros2"
        )
        self.assertIn(
            "my_listener_ros2",
            [s.node_name for s in subscribers],
            "Listener is not listening on expected topic",
        )

        ros2.shutdown("Test successful", timeout=None)
        self.assertFalse(ros2.is_running, "ROS2LaunchWrapper failed to shutdown")


if __name__ == "__main__":
    unittest.main()
