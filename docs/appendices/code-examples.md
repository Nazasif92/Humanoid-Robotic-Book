---
sidebar_label: "Appendix B: Code Examples"
---

# Appendix B: Code Examples

This appendix contains additional code examples that supplement the material in the main chapters.

## ROS 2 Publisher Node Template

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TemplatePublisher(Node):

    def __init__(self):
        super().__init__('template_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    template_publisher = TemplatePublisher()
    rclpy.spin(template_publisher)
    template_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## ROS 2 Subscriber Node Template

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TemplateSubscriber(Node):

    def __init__(self):
        super().__init__('template_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    template_subscriber = TemplateSubscriber()
    rclpy.spin(template_subscriber)
    template_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Launch File Example

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_nodes_py',
            executable='talker',
            name='my_node'
        )
    ])
```