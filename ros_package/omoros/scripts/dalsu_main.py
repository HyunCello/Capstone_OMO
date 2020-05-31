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
        if goalNo == 0:  # to 5Engineering
            self.msg.header.stamp = rospy.Time.now()
            self.msg.pose.position.x = -2.3
            self.msg.pose.position.y = 4.5
            self.msg.pose.orientation.z = 0.7
            self.msg.pose.orientation.w = 0.7
        elif goalNo == 1:
            self.msg.header.stamp = rospy.Time.now()
            self.msg.pose.position.x = -2.5
            self.msg.pose.position.y = 26.2
            self.msg.pose.orientation.z = 0.0
            self.msg.pose.orientation.w = 1.0
        
        if goalNo == 10:  # to Home from 5Engineering
            self.msg.header.stamp = rospy.Time.now()
            self.msg.pose.position.x = -2.5
            self.msg.pose.position.y = 20.3
            self.msg.pose.orientation.z = -0.7
            self.msg.pose.orientation.w = 0.7

        if goalNo == 20:  # to 3Engineering
            self.msg.header.stamp = rospy.Time.now()
            self.msg.pose.position.x = -1.8
            self.msg.pose.position.y = -4.0
            self.msg.pose.orientation.z = -0.7
            self.msg.pose.orientation.w = 0.7
        elif goalNo == 21:
            self.msg.header.stamp = rospy.Time.now()
            self.msg.pose.position.x = -1.7
            self.msg.pose.position.y = -9.5
            self.msg.pose.orientation.z = 0.0
            self.msg.pose.orientation.w = 1.0
        elif goalNo == 22:
            self.msg.header.stamp = rospy.Time.now()
            self.msg.pose.position.x = 16.0
            self.msg.pose.position.y = -8.5
            self.msg.pose.orientation.z = 0.0
            self.msg.pose.orientation.w = 1.0
        
        if goalNo == 30:  # to Home from 3Engineering
            self.msg.header.stamp = rospy.Time.now()
            self.msg.pose.position.x = 14.0
            self.msg.pose.position.y = -8.5
            self.msg.pose.orientation.z = 1.0
            self.msg.pose.orientation.w = 0.0
        elif goalNo == 31:
            self.msg.header.stamp = rospy.Time.now()
            self.msg.pose.position.x = -0.5
            self.msg.pose.position.y = -9.5
            self.msg.pose.orientation.z = 1.0
            self.msg.pose.orientation.w = 0.0
        elif goalNo == 32:
            self.msg.header.stamp = rospy.Time.now()
            self.msg.pose.position.x = -2.0
            self.msg.pose.position.y = -5.5
            self.msg.pose.orientation.z = 0.7
            self.msg.pose.orientation.w = 0.7

        if goalNo == 40:  # to 1Engineering
            self.msg.header.stamp = rospy.Time.now()
            self.msg.pose.position.x = -1.8
            self.msg.pose.position.y = -4.0
            self.msg.pose.orientation.z = -0.7
            self.msg.pose.orientation.w = 0.7
        elif goalNo == 41:
            self.msg.header.stamp = rospy.Time.now()
            self.msg.pose.position.x = -1.7
            self.msg.pose.position.y = -9.5
            self.msg.pose.orientation.z = 0.0
            self.msg.pose.orientation.w = 1.0
        elif goalNo == 42:
            self.msg.header.stamp = rospy.Time.now()
            self.msg.pose.position.x = 16.0
            self.msg.pose.position.y = -8.5
            self.msg.pose.orientation.z = 0.0
            self.msg.pose.orientation.w = 1.0

        if goalNo == 50:  # to Home from 1Engineering
            self.msg.header.stamp = rospy.Time.now()
            self.msg.pose.position.x = -1.8
            self.msg.pose.position.y = -4.0
            self.msg.pose.orientation.z = -0.7
            self.msg.pose.orientation.w = 0.7
        elif goalNo == 51:
            self.msg.header.stamp = rospy.Time.now()
            self.msg.pose.position.x = -1.7
            self.msg.pose.position.y = -9.5
            self.msg.pose.orientation.z = 0.0
            self.msg.pose.orientation.w = 1.0
        elif goalNo == 52:
            self.msg.header.stamp = rospy.Time.now()
            self.msg.pose.position.x = 16.0
            self.msg.pose.position.y = -8.5
            self.msg.pose.orientation.z = 0.0
            self.msg.pose.orientation.w = 1.0

        elif goalNo == 9999:  # home
            self.msg.header.stamp = rospy.Time.now()
            self.msg.pose.position.x = 0.0
            self.msg.pose.position.y = 0.0
            self.msg.pose.orientation.z = 0.0
            self.msg.pose.orientation.w = 1.0
        
        self.goal_pub.publish(self.msg)
        # rospy.loginfo('Goal No.' + str(goalNo) + ' published!')
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
    stat_pub = StatePublisher()
    pos_sub = GoalPosSubscriber()
    result_sub = ResultSubscriber()

    # test_pub = rospy.Publisher("lock", Int32, queue_size=1)
    # test_msg = Int32()
    # test_msg.data = 1

    rate = rospy.Rate(1) # 1hz
    rospy.loginfo("Dalsu_main Started")
    status = 1

    def dalsu_sleep(status):
        stat_pub.send_stat(status)
        rate.sleep()

    def wait_moving(status):
        while result_sub.wait_result() is False:
            dalsu_sleep(status)

    while not rospy.is_shutdown():
        goal_pos = pos_sub.wait_pos()

        ########## Move to Goal ##########
        if goal_pos is None:
            dalsu_sleep(status)
            continue
        
        elif goal_pos == 0:  # GoToHome_test
            status = 4
            goal_pub.send_goal(9999)
            wait_moving(status)
            status = 1
            rospy.loginfo('Arrived at goal ' + str(goal_pos))

        elif goal_pos == 1:  # 1Engineering
            status = 2
            goal_pub.send_goal(0)
            wait_moving(status)
            goal_pub.send_goal(1)
            wait_moving(status)
            rospy.loginfo('Arrived at goal ' + str(goal_pos))
            status = 3

        elif goal_pos == 3:
            status = 2
            goal_pub.send_goal(10)
            wait_moving(status)
            goal_pub.send_goal(11)
            wait_moving(status)
            rospy.loginfo('Arrived at goal ' + str(goal_pos))
            status = 3
        ########## Arrive at Goal ##########
        


if __name__ == '__main__':
    try:
        dalsu_main()
    except rospy.ROSInterruptException:
        pass