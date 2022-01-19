from actor_services.srv import SetPose
import rospy
import random
from time import sleep
from tf import TransformListener
import numpy as np

def get_point_at_distance(x0, y0, x1, y1, d):
    point0 = np.array([x0, y0])
    point1 = np.array([x1, y1])
    vec = point1-point0
    unit_vector = vec / np.linalg.norm(vec)
    my_point = point0 - d*unit_vector
    return [my_point[0], my_point[1]]


class PatientFollower:
    def __init__(self):
        self.tf = TransformListener()
        rospy.wait_for_service("/actor0/SetActorTarget")

    def follow_robot(self):
        try:
            position, _ = self.tf.lookupTransform("/odom", "/base_link", rospy.Time(0))
            x0, y0, _ = position
            position, _ = self.tf.lookupTransform("/odom", "/front_rocker_link", rospy.Time(0))
            x1, y1, _ = position
            # set_position = rospy.ServiceProxy('/actor0/SetActorPosition', SetPose)
            position = get_point_at_distance(x0, y0, x1, y1, 0.25)
            set_target = rospy.ServiceProxy('/actor5/SetActorTarget', SetPose)
            # resp = set_target(True, 0.0, 0.0)
            resp = set_target(True, position[0], position[1])
            # resp = set_position(True, position[0], position[1])
            print(resp)
        except Exception as e:
            print("Error: %s" %e)
            pass
        sleep(1)


if __name__ == "__main__":
    rospy.init_node("patient_controller")
    patient = PatientFollower()
    while True:
        patient.follow_robot()


