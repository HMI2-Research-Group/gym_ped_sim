import rospy
from geometry_msgs.msg import Twist


class SetRidgebackVelocity:
    def __init__(self):
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
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


if __name__ == "__main__":
    rospy.init_node("set_ridgeback_velocity")
    velocity = SetRidgebackVelocity()
    while True:
        velocity.set_velocity(0.5, 0.0)
        rospy.sleep(0.1)
