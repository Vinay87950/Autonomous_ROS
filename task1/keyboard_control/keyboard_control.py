#!/usr/bin/env python3
import sys
import threading
import tty
import termios

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

class ControlNode(Node):
    def __init__(self):
        super().__init__("control_node")

        # Set state for while loop
        self.alive = True

        # Print introduction
        self.printIntroduction()

        # Initialize Twist message
        self.twist = Twist()
        self.stop()

        self.get_logger().info("Initialize node for keyboard control.")

    def printIntroduction(self):
        msg = """
        This mode take keypresses from the keyboard and publishes them as Twist messages.
        -------------
        Moving around:
                w   
            a   s   d

        q/e : moving forward and rotating left/right at the same time

        z : terminate the program
        """

        print(msg)

    def stop(self):
        """Set the twist message to zero values."""
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0

    def getKey(self, settings):
        """Returns a string containing the key."""
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        self.get_logger().info(f"Key {key} get pressed.")
        return key 
    
    def setTwist(self, settings):
        """Set the twist variables to corresponding key. If 'z' is pressed, the alive status changes."""
        key = self.getKey(settings)
        self.stop()

        speed = 1.0
        turn = 5.0

        match key:
            case "w":
                self.twist.linear.x = speed
            case "s":
                self.twist.linear.x = -speed
            case "a":
                self.twist.angular.z = speed
            case "d":
                self.twist.angular.z = -speed
            case "q":
                self.twist.linear.x = speed
                self.twist.angular.z = turn
            case "e":
                self.twist.linear.x = speed
                self.twist.angular.z = -turn
            case "z":
                self.alive = False
                self.get_logger().info("Programm terminated.")
            case _:
                pass  

    def getTwist(self):
        return self.twist

def main(args=None):
    # Initialize ROS communications for a given context
    rclpy.init(args=args) 

    # Initialize own node
    node = ControlNode() 

    # Initialize publisher
    pub = node.create_publisher(Twist, "cmd_vel", 10)

    # Keep node alive
    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    try:
        # Save the terminal settings
        settings = termios.tcgetattr(sys.stdin)

        while(node.alive):
            # Move the robot
            node.setTwist(settings)
            msg = node.getTwist()
            pub.publish(msg)

    except Exception as e:
        print(e)

    finally:
        # Shutdown a previously initialized context
        rclpy.shutdown() 

        # Joining the spinner thread
        spinner.join()

        # Restore the terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()