#!/usr/bin/env python3

from collections import deque

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from matplotlib.animation import FuncAnimation
from person_msg.msg import PersonInfo
from rclpy.node import Node


class SpeedMonitorNode(Node):
    def __init__(self):
        super().__init__("speed_monitor")
        self.logger = self.get_logger()

        # Declare global logger level parameter
        self.declare_parameter("global_log_level", 20)  # Default to DEBUG level
        self.add_post_set_parameters_callback(self.parameters_callback)

        # Set initial log level from parameter
        self.logger.set_level(self.get_parameter("global_log_level").value)

        self.logger.info("Initializing Speed Monitor Node...")

        # Create subscription to person detection
        self.subscription = self.create_subscription(
            PersonInfo,
            "person_info",  # Topic name
            self.person_callback,
            10,  # QoS profile depth
        )
        self.logger.debug("Created subscription to 'person_info' topic")

        # Initialize speed parameters
        self.current_speed = 100.0  # Initial speed (km/h)
        self.target_speed = 100.0  # Target speed
        self.acceleration = 5.0  # Speed change rate (km/h per second)
        self.deceleration = -20.0  # Rapid deceleration when person detected

        # Initialize plotting
        plt.ion()  # Turn on interactive mode
        self.fig, self.ax = plt.subplots()  
        self.time_window = 10  # seconds to show in plot
        self.times = []  # Store all times
        self.speeds = []  # Store all speeds
        self.current_time = 0.0

        # Configure plot
        self.ax.set_ylim(0, 120)
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Speed (km/h)")
        self.ax.set_title("Vehicle Speed Monitor")
        self.ax.grid(True)

        # Initialize line object
        (self.line,) = self.ax.plot([], [], "b-", linewidth=2)

        self.logger.info("Speed Monitor Node successfully initialized")
        plt.show(block=False)
        plt.pause(0.1)  # Give the plot time to initialize

    def parameters_callback(self, parameters):
        """Callback for parameter changes"""
        for param in parameters:
            if param.name == "global_log_level":
                self.logger.set_level(param.value)
                self.logger.info(f"Log level changed to: {param.value}")

    def person_callback(self, msg):
        if msg.person_exists:
            self.target_speed = 0.0  # Stop when person detected
            self.logger.warn("Person detected! Emergency stop initiated.")
        else:
            self.logger.info("No person detected. Resuming normal speed.")
            self.target_speed = 100.0  # Resume normal speed

        # Calculate speed change
        if self.current_speed < self.target_speed:
            speed_change = self.acceleration * 0.05  # 0.05 seconds
        elif self.current_speed > self.target_speed:
            speed_change = self.deceleration * 0.05  # 0.05 seconds
        else:
            speed_change = 0.0  # Keep current speed if at target

        # Update current speed
        self.current_speed = max(0.0, min(100.0, self.current_speed + speed_change))

        # Log current speed
        self.logger.debug(
            f"Speed update - Current: {self.current_speed:.2f} km/h, Target: {self.target_speed:.2f} km/h, Change: {speed_change:.2f} km/h"
        )

        # Update plot data
        self.current_time += 0.05
        self.times.append(self.current_time)
        self.speeds.append(self.current_speed)

        # Update plot
        self.update_plot()

    def update_plot(self):
        # Update line data
        self.line.set_data(self.times, self.speeds)
        
        # Adjust x-axis limits to show last time_window seconds
        if self.current_time > self.time_window:
            self.ax.set_xlim(self.current_time - self.time_window, self.current_time)
        else:
            self.ax.set_xlim(0, self.time_window)
            
        # Redraw the plot
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)
    speed_monitor = SpeedMonitorNode()

    try:
        speed_monitor.logger.info("Starting Speed Monitor Node...")
        rclpy.spin(speed_monitor)
    except KeyboardInterrupt:
        speed_monitor.logger.info("Keyboard interrupt received, shutting down...")
    except Exception as e:
        speed_monitor.logger.error(f"An error occurred: {str(e)}")
    finally:
        speed_monitor.logger.info("Cleaning up resources...")
        speed_monitor.destroy_node()
        rclpy.shutdown()
        plt.close("all")


if __name__ == "__main__":
    main()
