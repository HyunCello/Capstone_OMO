#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult



def dalsu_main():
    rospy.init_node("dalsu_main", anonymous=False)
    goal_publisher = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=5)
    rospy.Subscriber("goalFlag", String, flag_callback)

    goal = PoseStamped()
    rate = rospy.Rate(10) # 10hz
    goal_flag = 0

    while not rospy.is_shutdown():
        goal = PoseStamped()

        if (goal_flag == 1):
            for i in range(5):
                set_goal(goal, i, 2.5, 1.1, -0.7, 0.7)
                goal_publisher.publish(goal)
                rospy.loginfo('Goal published!')
                rate.sleep()
            
            goal_flag = 0

        rate.sleep()
        #rospy.spin()


def set_goal(goal, i, x, y, z, w):
    goal.header.seq = i
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "map"

    goal.pose.position.x = 2.5
    goal.pose.position.y = 1.1
    goal.pose.position.z = 0.0

    goal.pose.orientation.x = 0.0
    goal.pose.orientation.y = 0.0
    goal.pose.orientation.z = -0.7
    goal.pose.orientation.w = 0.7


def flag_callback(data):
    rospy.loginfo("subscribed flag %s", data.data)



if __name__ == '__main__':
    try:
        dalsu_main()
    except rospy.ROSInterruptException:
        pass