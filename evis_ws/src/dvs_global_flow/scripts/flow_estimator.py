#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from dvs_msgs.msg import EventArray, Event
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
# import queue
import collections
from dvs_global_flow.warp_events import warp_events, Point2D

class FlowEstimator(Node):
    def __init__(self):
        super().__init__('flow_estimator')
        self.bridge = CvBridge()

        # Declare parameters
        self.declare_parameter("num_events_per_image", 5000)
        self.declare_parameter("num_events_slide", 5000)
        self.declare_parameter("contrast_measure", 0)
        self.declare_parameter("use_polarity", True)
        self.declare_parameter("smoothing_blur_sigma", 1.)

        # Subscribe to events
        self.create_subscription(EventArray, "/dvs/events", self.event_callback, 10)

        # publish global flow vector
        self.flow_pub = self.create_publisher(Point, "/global_flow", 10)
        # publish motion-compensated image
        self.img_pub = self.create_publisher(Image, "/mc_image", 10)

        self.event_queue = collections.deque()
        self.event_subset = []
        self.image_size = Point2D((0,0))

        self.first_event_id = 0
        self.publish_time = self.get_clock().now()
        
        self.get_logger().info("Node has started!")

    def event_callback(self, msg: EventArray):
        for e in msg.events:
            self.event_queue.append(e)

        if self.image_size[0] == 0:
            self.image_size = Point2D((msg.height, msg.width))

        packet_num = 0
        total_event_count = 0
        total_event_count += len(msg.events)

        num_events_per_iamge = self.get_parameter("num_events_per_image").get_parameter_value().integer_value
        while (len(self.event_queue) >= self.first_event_id + num_events_per_iamge):
            # Get subset of current window's events
            self.event_subset = list(self.event_queue)[:num_events_per_iamge]

            if packet_num == 0:
               # Find initial flow
               print("yes")
            packet_num += 1

            print(f"packet_num = {packet_num}, total_event_count = {total_event_count}")
            print(f"events = {len(self.event_queue)}, subs = {len(self.event_subset)}")

            # Maximise contrast 
            

            # Calculate time for publishing flow
            initial_time = Time.from_msg(self.event_subset[0].ts)
            last_time = Time.from_msg(self.event_subset[-1].ts)
            self.publish_time = Time(nanoseconds=0.5 * (initial_time.nanoseconds + last_time.nanoseconds))

            # Publish flow vector
            # self.flow_pub.publish()
            
            # # Publish motion compensated image
            # self.img_pub.publish()

            # slide window to next subset of events
            num_events_slide = self.get_parameter("num_events_slide").get_parameter_value().integer_value
            if (num_events_slide <= len(self.event_queue)):
                for i in range(num_events_slide):
                    self.event_queue.popleft()
                self.first_event_id += 0
            else:
                self.first_event_id += num_events_slide

def main(args=None):
    rclpy.init(args=args)
    node = FlowEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()