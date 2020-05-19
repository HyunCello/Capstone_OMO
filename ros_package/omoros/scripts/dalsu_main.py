#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult



class GoalPublisher():
    def __init__(self):
        self.goal_pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=1)
        self.msg = PoseStamped()

        #self.msg.header.seq = i
        self.msg.header.frame_id = "map"
        self.msg.pose.position.z = 0.0
        self.msg.pose.orientation.x = 0.0
        self.msg.pose.orientation.y = 0.0

    def send_goal(self, goalNo):
        if goalNo == 0:
            self.msg.header.stamp = rospy.Time.now()
            self.msg.pose.position.x = 0.2
            self.msg.pose.position.y = 1.0
            self.msg.pose.orientation.z = 0.7
            self.msg.pose.orientation.w = 0.7
        elif goalNo == 1:
            self.msg.header.stamp = rospy.Time.now()
            self.msg.pose.position.x = 2.5
            self.msg.pose.position.y = 1.1
            self.msg.pose.orientation.z = -0.7
            self.msg.pose.orientation.w = 0.7
        elif goalNo == 10:
            self.msg.header.stamp = rospy.Time.now()
            self.msg.pose.position.x = 0.2
            self.msg.pose.position.y = 1.0
            self.msg.pose.orientation.z = 1.0
            self.msg.pose.orientation.w = 0.0
        elif goalNo == 9999:  # home
            self.msg.header.stamp = rospy.Time.now()
            self.msg.pose.position.x = 0.0
            self.msg.pose.position.y = 0.0
            self.msg.pose.orientation.z = 0.0
            self.msg.pose.orientation.w = 1.0
        
        self.goal_pub.publish(self.msg)
        rospy.loginfo('Goal No.' + str(goalNo) + ' published!')
     


class FlagSubscriber():
    def __init__(self):
        self.flag_sub = rospy.Subscriber("goalFlag", Int32, callback=self._callback)
        self.flag_buf = None
    
    def wait_flag(self):
        if self.flag_buf is None:
            return None
        else:
            flag = self.flag_buf
            self.flag_buf = None
            return flag
    
    def _callback(self, msg):
        self.flag_buf = msg.data


class ResultSubscriber():
    def __init__(self):
        self.result_sub = rospy.Subscriber("move_base/result", MoveBaseActionResult, callback=self._callback)
        self.result_buf = False

    def wait_result(self):
        if self.result_buf is True:
            self.result_buf = False
            return True
        else:
            return False
    
    def _callback(self, msg):
        self.result_buf = (msg.status.text == 'Goal reached.')


def dalsu_main():
    rospy.init_node("dalsu_main", anonymous=False)
    goal_pub = GoalPublisher()
    flag_sub = FlagSubscriber()
    result_sub = ResultSubscriber()

    rate = rospy.Rate(10) # 10hz
    rospy.loginfo("Dalsu_main Started")

    while not rospy.is_shutdown():
        goal_flag = flag_sub.wait_flag()
        result = result_sub.wait_result()

        # if result is True:
        #     rospy.loginfo('Goal reached!')

        if goal_flag is None:
            rate.sleep()
            continue
        
        elif goal_flag == 0:  # GoToHome
            goal_pub.send_goal(9999)
            rate.sleep()

        elif goal_flag == 1:
            goal_pub.send_goal(0)
            rate.sleep()

            while result is False:
                result = result_sub.wait_result()
                rate.sleep()
            
            goal_pub.send_goal(1)
            rate.sleep()

        elif goal_flag == 2:
            goal_pub.send_goal(10)
            rate.sleep()

            while result is False:
                result = result_sub.wait_result()
                rate.sleep()
            
            goal_pub.send_goal(9999)
            rate.sleep()
        



if __name__ == '__main__':
    try:
        dalsu_main()
    except rospy.ROSInterruptException:
        pass