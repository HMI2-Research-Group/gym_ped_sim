from actor_services.srv import SetPose
import rospy
import random
from time import sleep
from tf import TransformListener


class PatientFollower:
    def __init__(self):
        self.tf = TransformListener()
        rospy.wait_for_service("/actor0/SetActorTarget")

    def follow_robot(self):
        try:
            position, _ = self.tf.lookupTransform("/odom", "/base_link", rospy.Time(0))
            # set_position = rospy.ServiceProxy('/actor0/SetActorPosition', SetPose)
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


