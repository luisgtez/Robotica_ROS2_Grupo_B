#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from person_msg.msg import PersonInfo
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
from collections import deque

class SpeedMonitorNode(Node):
    def __init__(self):
        super().__init__('speed_monitor')
        
        # Create subscription to person detection
        self.subscription = self.create_subscription(
            PersonInfo,
            'person_detection',  # Topic name
            self.person_callback,
            10  # QoS profile depth
        )
        
        # Initialize speed parameters
        self.current_speed = 100.0  # Initial speed (km/h)
        self.target_speed = 100.0   # Target speed
        self.acceleration = 5.0      # Speed change rate (km/h per second)
        self.deceleration = -20.0    # Rapid deceleration when person detected
        
        # Create timer for speed updates (50ms)
        self.update_timer = self.create_timer(0.05, self.update_speed)
        
        # Initialize plotting
        self.fig, self.ax = plt.subplots()
        self.time_window = 10  # seconds to show in plot
        self.times = deque(maxlen=int(self.time_window * 20))  # 20 points per second
        self.speeds = deque(maxlen=int(self.time_window * 20))
        self.current_time = 0.0
        
        # Configure plot
        self.ax.set_ylim(0, 120)
        self.ax.set_xlim(0, self.time_window)
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Speed (km/h)')
        self.ax.set_title('Vehicle Speed Monitor')
        self.ax.grid(True)
        
        # Initialize line object
        self.line, = self.ax.plot([], [], 'b-')
        
        # Start animation
        self.anim = FuncAnimation(
            self.fig, self.update_plot,
            interval=50,  # Update every 50ms
            blit=True
        )
        
        self.get_logger().info('Speed Monitor Node initialized')
        plt.show(block=False)
        
    def person_callback(self, msg):
        if msg.person_exists:
            self.target_speed = 0.0  # Stop when person detected
            self.get_logger().info('Person detected! Emergency stop initiated.')
        else:
            self.target_speed = 100.0  # Resume normal speed
            
    def update_speed(self):
        # Calculate speed change
        if self.current_speed < self.target_speed:
            speed_change = self.acceleration * 0.05  # 0.05 seconds
        else:
            speed_change = self.deceleration * 0.05  # 0.05 seconds
            
        # Update current speed
        self.current_speed = max(0.0, min(100.0, self.current_speed + speed_change))
        
        # Update plot data
        self.current_time += 0.05
        self.times.append(self.current_time)
        self.speeds.append(self.current_speed)
        
    def update_plot(self, frame):
        # Update line data
        self.line.set_data(
            np.array(self.times) - max(0, self.current_time - self.time_window),
            self.speeds
        )
        return self.line,

def main(args=None):
    rclpy.init(args=args)
    speed_monitor = SpeedMonitorNode()
    
    try:
        rclpy.spin(speed_monitor)
    except KeyboardInterrupt:
        pass
    finally:
        speed_monitor.destroy_node()
        rclpy.shutdown()
        plt.close('all')

if __name__ == '__main__':
    main() 