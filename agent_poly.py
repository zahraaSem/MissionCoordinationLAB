#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

from evry_project_plugins.srv import DistanceToFlag

from math import pi, sqrt, atan2, cos, sin
import numpy as np
from numpy import linalg as LA


class Robot:
    def __init__(self, robot_name):
        """Constructor of the class Robot
        The required publishers / subscribers are created.
        The attributes of the class are initialized

        Args:
            robot_name (str): Name of the robot, like robot_1, robot_2 etc. To be used for your subscriber and publisher with the robot itself
        """
        self.speed = 0.0
        self.angle = 0.0
        self.sonar = 0.0  # Sonar distance
        self.x, self.y = 0.0, 0.0  # coordinates of the robot
        self.yaw = 0.0  # yaw angle of the robot
        self.robot_name = robot_name

        '''Listener and publisher'''

        rospy.Subscriber(self.robot_name + "/sensor/sonar_front",
                         Range, self.callbackSonar)
        rospy.Subscriber(self.robot_name + "/odom",
                         Odometry, self.callbackPose)
        self.cmd_vel_pub = rospy.Publisher(
            self.robot_name + "/cmd_vel", Twist, queue_size=1)

    def callbackSonar(self, msg):
        """Callback function that gets the data coming from the ultrasonic sensor

        Args:
            msg (Range): Message that contains the distance separating the US sensor from a potential obstacle
        """
        self.sonar = msg.range

    def get_sonar(self):
        """Method that returns the distance separating the ultrasonic sensor from a potential obstacle
        """
        return self.sonar

    def callbackPose(self, msg):
        """Callback function that gets the data coming from the ultrasonic sensor

        Args:
            msg (Odometry): Message that contains the coordinates of the agent
        """
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        quaternion = msg.pose.pose.orientation
        quaternion_list = [quaternion.x,
                           quaternion.y, quaternion.z, quaternion.w]
        roll, pitch, yaw = euler_from_quaternion(quaternion_list)
        self.yaw = yaw

    def get_robot_pose(self):
        """Method that returns the position and orientation of the robot"""
        return self.x, self.y, self.yaw

    def constraint(self, val, min=-2.0, max=2.0):
        """Method that limits the linear and angular velocities sent to the robot

        Args:
            val (float): [Desired velocity to send
            min (float, optional): Minimum velocity accepted. Defaults to -2.0.
            max (float, optional): Maximum velocity accepted. Defaults to 2.0.

        Returns:
            float: Limited velocity whose value is within the range [min; max]
        """
        # DO NOT TOUCH
        if val < min:
            return min
        if val > max:
            return max
        return val

    def set_speed_angle(self, linear, angular):
        """Method that publishes the proper linear and angular velocities commands on the related topic to move the robot

        Args:
            linear (float): desired linear velocity
            angular (float): desired angular velocity
        """
        cmd_vel = Twist()
        cmd_vel.linear.x = self.constraint(linear)
        cmd_vel.angular.z = self.constraint(angular, min=-1, max=1)
        self.cmd_vel_pub.publish(cmd_vel)

    def getDistanceToFlag(self):
        """Get the distance separating the agent from a flag. The service 'distanceToFlag' is called for this purpose.
        The current position of the robot and its id should be specified. The id of the robot corresponds to the id of the flag it should reach


        Returns:
            float: the distance separating the robot from the flag
        """
        rospy.wait_for_service('/distanceToFlag')
        try:
            service = rospy.ServiceProxy('/distanceToFlag', DistanceToFlag)
            pose = Pose2D()
            pose.x = self.x
            pose.y = self.y
            # int(robot_name[-1]) corresponds to the id of the robot. It is also the id of the related flag
            result = service(pose, int(self.robot_name[-1]))
            return result.distance
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def get_polynomial_time_scaling_3rd_order(self, pose_start, v_start, p_end, v_end, T):
        # input: p,v: position and velocity of start/end point
        #        T: the desired time to complete this segment of trajectory (in second)
        M = np.array([[0, 0, 0, 1], [T**3, T**2, T, 1],
                     [0, 0, 1, 0], [3*T**2, 2*T, 1, 0]])
        Mt = M.transpose()
        X = np.array([pose_start, v_start, p_end, v_end])
        [a, b, c, d] = Mt.dot(X)
        # output: the coefficients of this polynomial
        return a, b, c, d


def run_demo():
    """Main loop"""
    robot_name = rospy.get_param("~robot_name")
    robot = Robot(robot_name)
    print(f"Robot : {robot_name} is starting..")
    #delay = 0.1
    # Timing
    #rospy.sleep(delay + int(robot_name[-1]))

    # strategy 2
    velocity = 2
    angle = 0
    sonar = float(robot.get_sonar())
    distance = float(robot.getDistanceToFlag())
    print(f"{robot_name} distance to flag = ", distance)
    print(f"{robot_name} distance to obstacle = ", sonar)
    PI = 3.141592654
    K_a = 0.5
    K_l = 0.5
    angleTolerance = 0.1

    if(robot.robot_name == 'robot_1'):
        pathx_a, pathx_b, pathx_c, pathx_d = robot.get_polynomial_time_scaling_3rd_order(
            robot.x, 2, -21.21320344, 0, 0.5)
        pathy_a, pathy_b, pathy_c, pathy_d = robot.get_polynomial_time_scaling_3rd_order(
            robot.y, 2, 21.21320344, 0, 0.5)
        angle_robot = 0.15
    elif(robot.robot_name == 'robot_2'):
        pathx_a, pathx_b, pathx_c, pathx_d = robot.get_polynomial_time_scaling_3rd_order(
            robot.x, 2, 21.21320344, 0, 0.5)
        pathy_a, pathy_b, pathy_c, pathy_d = robot.get_polynomial_time_scaling_3rd_order(
            robot.y, 2, 21.21320344, 0, 0.5)
        angle_robot = 0
    else:
        pathx_a, pathx_b, pathx_c, pathx_d = robot.get_polynomial_time_scaling_3rd_order(
            robot.x, 2, 0, 0, 0.5)
        pathy_a, pathy_b, pathy_c, pathy_d = robot.get_polynomial_time_scaling_3rd_order(
            robot.y, 2, -30, 0, 0.5)
        angle_robot = -0.1

    t = 0

    while not rospy.is_shutdown():

        # Write here your strategy.
        print(f"time :{t}")
        print(
            f"Distance to flag robot {robot.robot_name}: {robot.getDistanceToFlag()}")
        print(f"Distance sonor {robot.robot_name}:{robot.get_sonar()}")
        sonar = float(robot.get_sonar())
        distance = float(robot.getDistanceToFlag())
        t = t+0.1

        X = pathx_a*t**3 + pathx_b*t**2 + pathx_c*t + pathx_d
        Y = pathy_a*t**3 + pathy_b*t**2 + pathy_c*t + pathy_d
        dX = 3*pathx_a*t**2 + 2*pathx_b*t + pathx_c
        dY = 3*pathy_a*t**2 + 2*pathy_b*t + pathy_c
        velocity = sqrt(dX**2 + dY**2)
        deltaX = X - robot.x
        deltaY = Y - robot.y
        waypointHeading = atan2(deltaY, deltaX)
        headingError = waypointHeading - robot.yaw

        if (distance < 2):
            velocity = 0
            angle = 0
            robot.set_speed_angle(velocity, angle)
        else:
            if (headingError > PI):
                headingError = headingError - (2 * PI)
            elif (headingError < -PI):
                headingError = headingError + (2 * PI)

            if (abs(headingError) > angleTolerance):
                Velocity = 0.0
                angle = K_a * headingError + angle_robot
            else:
                velocity = K_l*distance
                angle = angle_robot

        robot.set_speed_angle(velocity, angle)

        # Finishing by publishing the desired speed.
        # DO NOT TOUCH.
        robot.set_speed_angle(velocity, angle)
        rospy.sleep(0.5)


if __name__ == "__main__":
    print("Running ROS..")
    rospy.init_node("Controller", anonymous=True)
    run_demo()
