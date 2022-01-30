import rospy
from actor_services.srv import SetHumanHeading
from tf import TransformListener
import numpy as np


class PatientFollower:
    def __init__(self):
        self.tf = TransformListener()
        rospy.wait_for_service("/actor5/SetActorHeading")

    def follow_robot(self):
        set_heading = rospy.ServiceProxy("/actor5/SetActorHeading", SetHumanHeading)
        set_heading(0.0, 0.0, np.pi/2)


if __name__ == "__main__":
    rospy.init_node("test_node1")
    patient = PatientFollower()
    for _ in range(3):
        patient.follow_robot()
