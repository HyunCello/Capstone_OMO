#!/usr/bin/env python
######## last update : 20.05.21 18:00 by kyp
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
        # in turtlebot
        # if goalNo == 0:
        #     self.msg.header.stamp = rospy.Time.now()
        #     self.msg.pose.position.x = 0.2
        #     self.msg.pose.position.y = 1.0
        #     self.msg.pose.orientation.z = 0.7
        #     self.msg.pose.orientation.w = 0.7
        # elif goalNo == 1:
        #     self.msg.header.stamp = rospy.Time.now()
        #     self.msg.pose.position.x = 2.5
        #     self.msg.pose.position.y = 1.1
        #     self.msg.pose.orientation.z = -0.7
        #     self.msg.pose.orientation.w = 0.7
        # elif goalNo == 10:
        #     self.msg.header.stamp = rospy.Time.now()
        #     self.msg.pose.position.x = 0.2
        #     self.msg.pose.position.y = 1.0
        #     self.msg.pose.orientation.z = 1.0
        #     self.msg.pose.orientation.w = 0.0

        # in dalsu
        if goalNo == 0:
            self.msg.header.stamp = rospy.Time.now()
            self.msg.pose.position.x = 1.3
            self.msg.pose.position.y = -2.2
            self.msg.pose.orientation.z = 0.05
            self.msg.pose.orientation.w = 1.0
        elif goalNo == 1:
            self.msg.header.stamp = rospy.Time.now()
            self.msg.pose.position.x = 4.0
            self.msg.pose.position.y = -1.9
            self.msg.pose.orientation.z = 0.74
            self.msg.pose.orientation.w = 0.67
        elif goalNo == 2:
            self.msg.header.stamp = rospy.Time.now()
            self.msg.pose.position.x = 3.35
            self.msg.pose.position.y = 3.69
            self.msg.pose.orientation.z = 1.0
            self.msg.pose.orientation.w = -0.07
        
        if goalNo == 10:
            self.msg.header.stamp = rospy.Time.now()
            self.msg.pose.position.x = -0.17
            self.msg.pose.position.y = 3.12
            self.msg.pose.orientation.z = 0.09
            self.msg.pose.orientation.w = 1.0
        elif goalNo == 11:
            self.msg.header.stamp = rospy.Time.now()
            self.msg.pose.position.x = 3.0
            self.msg.pose.position.y = 3.75
            self.msg.pose.orientation.z = -0.65
            self.msg.pose.orientation.w = 0.76
        

        elif goalNo == 9999:  # home
            self.msg.header.stamp = rospy.Time.now()
            self.msg.pose.position.x = 0.0
            self.msg.pose.position.y = 0.0
            self.msg.pose.orientation.z = 0.0
            self.msg.pose.orientation.w = 1.0
        
        self.goal_pub.publish(self.msg)
        rospy.loginfo('Goal No.' + str(goalNo) + ' published!')
        rospy.Rate(10).sleep()


class StatePublisher():
    def __init__(self):
        self.stat_pub = rospy.Publisher("robotStatus", Int32, queue_size=1)
        self.msg = Int32()

    # stat : 1 - wait for delivery, 2 - in delivery, 3 - wait for complete, 4 - in return
    def send_stat(self, stat):
        self.msg.data = stat
        self.stat_pub.publish(self.msg)
        rospy.loginfo('state :  ' + str(stat) + ' published. ')

        if stat == 1:
            rospy.loginfo('wait for delivery... ')
        if stat == 2:
            rospy.loginfo('in delivery... ')
        if stat == 3:
            rospy.loginfo('arrived at goal and waiting... ')
        if stat == 4:
            rospy.loginfo('in return... ')


class GoalPosSubscriber():
    def __init__(self):
        self.pos_sub = rospy.Subscriber("goalPos", Int32, callback=self._callback)
        self.pos_buf = None
    
    def wait_pos(self):
        if self.pos_buf is None:
            return None
        else:
            pos = self.pos_buf
            self.pos_buf = None
            return pos
    
    def _callback(self, msg):
        self.pos_buf = msg.data


class ResultSubscriber():
    def __init__(self):
        self.result_sub = rospy.Subscriber("move_base/result", MoveBaseActionResult, callback=self._callback)
        self.result_buf = False

    def wait_result(self):
        while self.result_buf is False:
            rospy.Rate(10).sleep()
        self.result_buf = False
    
    def _callback(self, msg):
        self.result_buf = (msg.status.text == 'Goal reached.')


def dalsu_main():
    rospy.init_node("dalsu_main", anonymous=False)
    goal_pub = GoalPublisher()
    stat_pub = StatePublisher()
    pos_sub = GoalPosSubscriber()
    result_sub = ResultSubscriber()

    test_pub = rospy.Publisher("lock", Int32, queue_size=1)
    test_msg = Int32()
    test_msg.data = 1

    rate = rospy.Rate(1) # 10hz
    rospy.loginfo("Dalsu_main Started")
    stat_pub.send_stat(1)

    while not rospy.is_shutdown():
        goal_pos = pos_sub.wait_pos()

        test_pub.publish(test_msg)

        ########## Move to Goal ##########
        if goal_pos is None:
            rate.sleep()
            continue
        
        elif goal_pos == 0:  # GoToHome
            goal_pub.send_goal(9999)
            result_sub.wait_result()
            rospy.loginfo('Arrived at goal ' + str(goal_pos))

        elif goal_pos == 1:
            stat_pub.send_stat(2)
            goal_pub.send_goal(0)
            result_sub.wait_result()
            goal_pub.send_goal(1)
            result_sub.wait_result()
            goal_pub.send_goal(2)
            result_sub.wait_result()
            rospy.loginfo('Arrived at goal ' + str(goal_pos))
            stat_pub.send_stat(3)

        elif goal_pos == 2:
            stat_pub.send_stat(2)
            goal_pub.send_goal(10)
            result_sub.wait_result()
            goal_pub.send_goal(11)
            result_sub.wait_result()
            rospy.loginfo('Arrived at goal ' + str(goal_pos))
            stat_pub.send_stat(3)
        ########## Arrive at Goal ##########
        


if __name__ == '__main__':
    try:
        dalsu_main()
    except rospy.ROSInterruptException:
        pass