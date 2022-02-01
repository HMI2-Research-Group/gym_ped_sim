from actor_services.srv import SetPose
from actor_services.srv import SetHumanHeading
import rospy
import random
from time import sleep
from tf import TransformListener
import numpy as np
from geometry_msgs.msg import Twist


def get_point_at_distance(x0, y0, x1, y1, d):
    point0 = np.array([x0, y0])
    point1 = np.array([x1, y1])
    vec = point1 - point0
    unit_vector = vec / np.linalg.norm(vec)
    my_point = point0 - d * unit_vector
    return [my_point[0], my_point[1]], np.arctan(unit_vector[1] / unit_vector[0])


class SetRidgebackVelocity:
    def __init__(self):
        self.pub = rospy.Publisher("/ridgeback_velocity_controller/cmd_vel", Twist, queue_size=1)
        self.twistmsg = Twist()

    def set_velocity(self, x, y):
        # Set's the forwarding moving velocity
        # For sideways, look at this snippet
        # https://github.com/heracleia/panda_gui/blob/main/src/thesis2.py#L48
        # self.stop()
        # This will enable the summit to move forward in the direction we want
        # x *= -1
        # y *= -1
        self.twistmsg.linear.x = x
        self.twistmsg.linear.y = y
        self.pub.publish(self.twistmsg)

    def stop(self):
        # The argument to use this function is that, the robot motion is not immediate unless you cease it
        # using this function
        self.twistmsg.linear.x = 0.0
        self.twistmsg.linear.y = 0.0
        self.twistmsg.linear.z = 0.0
        self.twistmsg.angular.x = 0.0
        self.twistmsg.angular.y = 0.0
        self.twistmsg.angular.z = 0.0
        self.pub.publish(self.twistmsg)


class PatientFollower:
    def __init__(self):
        self.tf = TransformListener()
        rospy.wait_for_service("/actor5/SetActorTarget")
        rospy.wait_for_service("/actor5/SetActorHeading")
        rospy.wait_for_service("/actor5/SetActorPosition")
        self.patient_robot_distance = 2.0
        self.set_patient_initial_position()

    def set_patient_initial_position(self):
        i = 0
        while True:
            try:
                patient_traget, heading = self.patient_target()
                set_position = rospy.ServiceProxy("/actor5/SetActorPosition", SetPose)
                _ = set_position(True, patient_traget[0], patient_traget[1])
                set_heading = rospy.ServiceProxy("/actor5/SetActorHeading", SetHumanHeading)
                set_heading(0.0, 0.0, heading)
                i += 1
                if i > 3:
                    break
            except Exception as e:
                print("Error: %s" % e)
                pass

    def patient_target(self):
        try:
            position, _ = self.tf.lookupTransform("/odom", "/rear_rocker_link", rospy.Time(0))
            x0, y0, _ = position
            position, _ = self.tf.lookupTransform("/odom", "/front_rocker_link", rospy.Time(0))
            x1, y1, _ = position
            target, heading_angle = get_point_at_distance(x0, y0, x1, y1, self.patient_robot_distance)
            return target, heading_angle
        except Exception as e:
            print("Error: %s" % e)
            pass

    def follow_robot(self):
        try:
            position, heading_angle = self.patient_target()
            set_target = rospy.ServiceProxy("/actor5/SetActorTarget", SetPose)
            # resp = set_target(True, 0.0, 0.0)
            resp = set_target(True, position[0], position[1])
            # resp = set_position(True, position[0], position[1])
            current_position = np.array([resp.x, resp.y])
            dist = np.linalg.norm(current_position - np.array(position))
            if dist < 1e-4:
                set_heading = rospy.ServiceProxy("/actor5/SetActorHeading", SetHumanHeading)
                set_heading(0.0, 0.0, heading_angle)
            print(dist)
        except Exception as e:
            print("Error: %s" % e)
            pass


if __name__ == "__main__":
    rospy.init_node("patient_controller")
    patient = PatientFollower()
    set_ridgeback_vel = SetRidgebackVelocity()
    i = 0
    while True:
        patient.follow_robot()
        set_ridgeback_vel.set_velocity(0.5, 0.0)
        rospy.sleep(0.1)
        i += 1
        if i > 1e3:
            break
