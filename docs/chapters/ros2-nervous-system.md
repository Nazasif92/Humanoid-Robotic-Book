---
sidebar_position: 2
title: "Module 1: ROS 2 — Robotic Nervous System"
---

# Module 1: ROS 2 — Robotic Nervous System

## Introduction

Welcome to the foundational module of the Humanoid Robotics Book. In this chapter, you'll learn about ROS 2 (Robot Operating System 2), which serves as the communication backbone for humanoid robots. Think of ROS 2 as the nervous system of a robot—connecting different parts and enabling them to communicate effectively.

ROS 2 is essential for humanoid robotics because it provides the infrastructure for different robot components to work together. Whether your robot needs to process sensor data, plan movements, or execute actions, ROS 2 provides the communication layer that makes it all possible.

## Core Concepts

### What is a Node?

A node in ROS 2 is a single executable process that performs a specific task. Think of nodes as individual organs in a body—each has a specific function but works together as part of a larger system. For example, one node might handle camera input, another might process that input to detect objects, and a third might send commands to move the robot.

In ROS 2, nodes are designed to be modular and reusable. This means you can create a node that performs a specific function and use it in different robot applications without modification.

### Topics and Publisher-Subscriber Communication

Topics are ROS 2's primary method of communication between nodes. The publisher-subscriber pattern allows nodes to send and receive data asynchronously. A publisher node sends messages to a topic, and any number of subscriber nodes can listen to that topic to receive the messages.

This communication pattern is ideal for robotics because it decouples nodes from each other. A camera node doesn't need to know which other nodes are using its data—it simply publishes to a topic. Similarly, any node that needs camera data can subscribe to that topic without the publisher needing to know about it.

### Services and Request-Response Communication

While topics are great for continuous data streams, sometimes you need a request-response interaction. Services in ROS 2 provide this synchronous communication pattern. A client node sends a request to a service, and the service node processes the request and sends back a response.

Services are useful for operations that need a specific result, like asking a navigation system to compute a path to a destination or requesting a robot to move to a specific location.

### Actions for Long-Running Tasks

Actions are used for tasks that take a long time to complete and may need to be monitored or canceled. They combine the features of topics and services, providing feedback during execution, goal status updates, and the ability to cancel operations.

Actions are perfect for complex robot behaviors like moving to a location (which takes time and may need to be interrupted) or manipulating objects (which involves multiple steps and feedback).

### Quality of Service (QoS)

Quality of Service settings in ROS 2 allow you to fine-tune communication behavior between nodes. You can specify how important it is to receive all messages, how long messages should be kept in queues, and how nodes should behave when the network is unreliable.

For humanoid robots, QoS settings are crucial because they help ensure reliable communication even in challenging environments.

## Your First ROS 2 Package

Let's create your first ROS 2 package to understand how everything works together. We'll create a simple publisher node that sends messages to a topic.

```python
# publisher_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
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
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

This simple publisher node demonstrates the basic structure of a ROS 2 node. It creates a publisher that sends "Hello World" messages to a topic every 0.5 seconds. You can run this node and use `ros2 topic echo /topic` to see the messages it publishes.

## URDF for Humanoid Robots

URDF (Unified Robot Description Format) is XML-based format used to describe robot models in ROS. For humanoid robots, URDF defines the physical structure: links (body parts), joints (connections), and properties like mass and inertia.

A simple humanoid URDF might define a torso, two arms, two legs, and a head, with appropriate joints connecting them. This description is essential for simulation, visualization, and motion planning.

```xml
<!-- Example URDF snippet -->
<robot name="simple_humanoid">
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>
</robot>
```

## Launch Files and Integration

Launch files in ROS 2 allow you to start multiple nodes at once with predefined parameters. This is essential for humanoid robots, which typically require many nodes running simultaneously to function properly.

A launch file can start perception nodes, control nodes, and communication nodes all with the correct parameters and configurations, making it easy to run complex robot behaviors.

## Summary

In this module, you've learned the fundamentals of ROS 2:
- Nodes as the basic computational units
- Topics for asynchronous communication
- Services for request-response interactions
- Actions for long-running tasks
- How to create a simple publisher node
- The basics of URDF for robot description
- The role of launch files in integration

These concepts form the foundation for all the modules that follow. In the next module, you'll learn how to simulate these ROS 2 concepts in a digital twin environment using Gazebo and Unity.