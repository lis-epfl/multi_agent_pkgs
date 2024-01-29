#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
import numpy as np


class TFDifferenceCalculator(Node):
    def __init__(self):
        super().__init__('tf_difference_calculator')

        self.agent_frame = 'agent_2'
        self.cf_frame = 'cf2'

        self.agent_2_ready = False
        self.cf_ready = False
        self.last_agent_transform = None
        self.last_cf_transform = None
        self.distances = []

        # Subscribe to the TF messages
        self.subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10  # Adjust the queue size as needed
        )
        self.subscription  # prevent unused variable warning

    def tf_callback(self, tf_msg):
        for transform in tf_msg.transforms:
            # Check if the transform is between the desired frames
            if transform.child_frame_id == self.agent_frame:
                self.agent_2_ready = True
                if self.cf_ready:
                    # Calculate distance
                    x_diff = self.last_agent_transform.transform.translation.x - \
                        transform.transform.translation.x
                    y_diff = self.last_agent_transform.transform.translation.y - \
                        transform.transform.translation.y
                    z_diff = self.last_agent_transform.transform.translation.z - \
                        transform.transform.translation.z
                    distance = np.sqrt(x_diff**2 + y_diff**2 + z_diff**2)

                    # Store distance for later calculations
                    self.distances.append(distance)

                    # Display results
                    self.get_logger().info('Current Distance: %.3f meters' % distance)
                    self.get_logger().info('Average Distance: %.3f meters' % np.mean(self.distances))
                    self.get_logger().info('Standard Deviation: %.3f meters' % np.std(self.distances))
                    self.get_logger().info('Maximum Distance: %.3f meters' % np.max(self.distances))

                    self.cf_read = False

            elif transform.child_frame_id == self.cf_frame:
                # Check if the agent_2 transform is ready
                if self.agent_2_ready:
                    # set cf_ready to true
                    self.cf_ready = True
                    self.agent_2_ready = False
                    self.last_agent_transform = transform


def main(args=None):
    rclpy.init(args=args)
    tf_difference_calculator = TFDifferenceCalculator()
    rclpy.spin(tf_difference_calculator)
    tf_difference_calculator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
