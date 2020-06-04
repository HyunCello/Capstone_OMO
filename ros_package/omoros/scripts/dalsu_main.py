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
        # if goalNo == 40:
        #     self.msg.header.stamp = rospy.Time.now()
        #     self.msg.pose.position.x = 0.2
        #     self.msg.pose.position.y = 1.0
        #     self.msg.pose.orientation.z = 0.7
        #     self.msg.pose.orientation.w = 0.7
        # elif goalNo == 41:
        #     self.msg.header.stamp = rospy.Time.now()
        #     self.msg.pose.position.x = 2.5
        #     self.msg.pose.position.y = 1.1
        #     self.msg.pose.orientation.z = -0.7
        #     self.msg.pose.orientation.w = 0.7
        # elif goalNo == 50:
        #     self.msg.header.stamp = rospy.Time.now()
        #     self.msg.pose.position.x = 0.2
        #     self.msg.pose.position.y = 1.0
        #     self.msg.pose.orientation.z = 1.0
        #     self.msg.pose.orientation.w = 0.0

        # in dalsu
        if goalNo == 0:  # to 1Engineering
            self.msg.header.stamp = rospy.Time.now()
            self.msg.pose.position.x = -1.28
            self.msg.pose.position.y = -9.06
            self.msg.pose.orientation.z = -0.7
            self.msg.pose.orientation.w = 0.7
        elif goalNo == 1:
            self.msg.header.stamp = rospy.Time.now()
            self.msg.pose.position.x = -0.27
            self.msg.pose.position.y = -28.33
            self.msg.pose.orientation.z = 1.0
            self.msg.pose.orientation.w = 0.0
        elif goalNo == 2:
            self.msg.header.stamp = rospy.Time.now()
            self.msg.pose.position.x = -25.65
            self.msg.pose.position.y = -31.1
            self.msg.pose.orientation.z = 1.0
            self.msg.pose.orientation.w = 0.0
        elif goalNo == 3:
            self.msg.header.stamp = rospy.Time.now()
            self.msg.pose.position.x = -24.71
            self.msg.pose.position.y = -51.61
            self.msg.pose.orientation.z = -0.7
            self.msg.pose.orientation.w = 0.7

        if goalNo == 10:  # to Home from 1Engineering
            self.msg.header.stamp = rospy.Time.now()
            self.msg.pose.position.x = -25.33
            self.msg.pose.position.y = -44.46
            self.msg.pose.orientation.z = 0.7
            self.msg.pose.orientation.w = 0.7
        elif goalNo == 11:
            self.msg.header.stamp = rospy.Time.now()
            self.msg.pose.position.x = -24.65
            self.msg.pose.position.y = -31.1
            self.msg.pose.orientation.z = 0.0
            self.msg.pose.orientation.w = 1.0
        elif goalNo == 12:
            self.msg.header.stamp = rospy.Time.now()
            self.msg.pose.position.x = -0.27
            self.msg.pose.position.y = -28.33
            self.msg.pose.orientation.z = 0.0
            self.msg.pose.orientation.w = 1.0
        elif goalNo == 13:
            self.msg.header.stamp = rospy.Time.now()
            self.msg.pose.position.x = -1.28
            self.msg.pose.position.y = -9.06
            self.msg.pose.orientation.z = 0.7
            self.msg.pose.orientation.w = 0.7

        if goalNo == 20:  # to 3Engineering
            self.msg.header.stamp = rospy.Time.now()
            self.msg.pose.position.x = -1.28
            self.msg.pose.position.y = -9.06
            self.msg.pose.orientation.z = -0.7
            self.msg.pose.orientation.w = 0.7
        elif goalNo == 21:
            self.msg.header.stamp = rospy.Time.now()
            self.msg.pose.position.x = 10.32
            self.msg.pose.position.y = -8.92
            self.msg.pose.orientation.z = 0.0
            self.msg.pose.orientation.w = 1.0
        elif goalNo == 22:
            self.msg.header.stamp = rospy.Time.now()
            self.msg.pose.position.x = 18.0
            self.msg.pose.position.y = -8.9
            self.msg.pose.orientation.z = -0.7
            self.msg.pose.orientation.w = 0.7
        elif goalNo == 23:
            self.msg.header.stamp = rospy.Time.now()
            self.msg.pose.position.x = 18.23
            self.msg.pose.position.y = -25.11
            self.msg.pose.orientation.z = -0.7
            self.msg.pose.orientation.w = 0.7
        elif goalNo == 24:
            self.msg.header.stamp = rospy.Time.now()
            self.msg.pose.position.x = 28.78
            self.msg.pose.position.y = -26.08
            self.msg.pose.orientation.z = 0.0
            self.msg.pose.orientation.w = 1.0
        
        if goalNo == 30:  # to Home from 3Engineering
            self.msg.header.stamp = rospy.Time.now()
            self.msg.pose.position.x = 24.88
            self.msg.pose.position.y = -26.18
            self.msg.pose.orientation.z = 1.0
            self.msg.pose.orientation.w = 0.0
        elif goalNo == 31:
            self.msg.header.stamp = rospy.Time.now()
            self.msg.pose.position.x = 18.23
            self.msg.pose.position.y = -25.11
            self.msg.pose.orientation.z = 0.7
            self.msg.pose.orientation.w = 0.7
        elif goalNo == 32:
            self.msg.header.stamp = rospy.Time.now()
            self.msg.pose.position.x = 18.0
            self.msg.pose.position.y = -8.9
            self.msg.pose.orientation.z = 0.7
            self.msg.pose.orientation.w = 0.7
        elif goalNo == 33:
            self.msg.header.stamp = rospy.Time.now()
            self.msg.pose.position.x = 10.32
            self.msg.pose.position.y = -8.92
            self.msg.pose.orientation.z = 1.0
            self.msg.pose.orientation.w = 0.0
        elif goalNo == 34:
            self.msg.header.stamp = rospy.Time.now()
            self.msg.pose.position.x = -1.28
            self.msg.pose.position.y = -9.06
            self.msg.pose.orientation.z = 0.7
            self.msg.pose.orientation.w = 0.7

        if goalNo == 40:  # to 5Engineering
            self.msg.header.stamp = rospy.Time.now()
            self.msg.pose.position.x = -2.23
            self.msg.pose.position.y = 5.94
            self.msg.pose.orientation.z = 0.7
            self.msg.pose.orientation.w = 0.7
        elif goalNo == 41:
            self.msg.header.stamp = rospy.Time.now()
            self.msg.pose.position.x = -2.56
            self.msg.pose.position.y = 17.0
            self.msg.pose.orientation.z = 0.7
            self.msg.pose.orientation.w = 0.7
        elif goalNo == 42:
            self.msg.header.stamp = rospy.Time.now()
            self.msg.pose.position.x = -2.85
            self.msg.pose.position.y = 24.85
            self.msg.pose.orientation.z = 0.0
            self.msg.pose.orientation.w = 1.0
        
        if goalNo == 50:  # to Home from 5Engineering
            self.msg.header.stamp = rospy.Time.now()
            self.msg.pose.position.x = -2.56
            self.msg.pose.position.y = 17.0
            self.msg.pose.orientation.z = -0.7
            self.msg.pose.orientation.w = 0.7
        elif goalNo == 51:
            self.msg.header.stamp = rospy.Time.now()
            self.msg.pose.position.x = -2.23
            self.msg.pose.position.y = 5.94
            self.msg.pose.orientation.z = -0.7
            self.msg.pose.orientation.w = 0.7

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

        # if stat == 1:
        #     rospy.loginfo('wait for delivery... ')
        # if stat == 2:
        #     rospy.loginfo('in delivery... ')
        # if stat == 3:
        #     rospy.loginfo('arrived at goal and waiting... ')
        # if stat == 4:
        #     rospy.loginfo('in return... ')


class GoalPosPublisher():
    def __init__(self):
        self.pos_pub = rospy.Publisher("goalPosPrev", Int32, queue_size=1)
        self.msg = Int32()

    def send_pos(self, pos):
        self.msg.data = pos
        self.pos_pub.publish(self.msg)


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


class DoorStatusSubscriber():
    def __init__(self):
        self.door_sub = rospy.Subscriber("doorStatus", Int32, callback=self._callback)
        self.door_buf = None
    
    def wait_door(self):
        if self.door_buf == 3:
            self.door_buf = None
            return True
        else:
            self.door_buf = None
            return False
    
    def _callback(self, msg):
        self.door_buf = msg.data


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
    pos_pub = GoalPosPublisher()
    pos_sub = GoalPosSubscriber()
    door_sub = DoorStatusSubscriber()
    result_sub = ResultSubscriber()

    # test_pub = rospy.Publisher("lock", Int32, queue_size=1)
    # test_msg = Int32()
    # test_msg.data = 1

    rate = rospy.Rate(1) # 1hz
    rospy.loginfo("Dalsu_main Started")
    status = 1

    def dalsu_sleep(status, pos):
        stat_pub.send_stat(status)
        pos_pub.send_pos(pos)
        rate.sleep()

    def wait_moving(status, pos):
        while result_sub.wait_result() is False:
            dalsu_sleep(status, pos)

    def wait_closeDoor(status, pos):
        while door_sub.wait_door() is False:
            dalsu_sleep(status, pos)

    while not rospy.is_shutdown():
        goal_pos = pos_sub.wait_pos()

        ########## Move to Goal ##########
        if goal_pos is None:
            dalsu_sleep(status, 0)
            continue
        
        elif goal_pos == 0:  # GoToHome_test (Emergency Return)
            status = 4
            goal_pub.send_goal(9999)
            wait_moving(status, goal_pos)
            status = 1
            rospy.loginfo('Arrived at Home')

        elif goal_pos == 1:  # 1st_Engineering
            status = 2
            wait_closeDoor(status, goal_pos)
            goal_pub.send_goal(0)
            wait_moving(status, goal_pos)
            goal_pub.send_goal(1)
            wait_moving(status, goal_pos)
            goal_pub.send_goal(2)
            wait_moving(status, goal_pos)
            goal_pub.send_goal(3)
            wait_moving(status, goal_pos)
            rospy.loginfo('Arrived at Goal ' + str(goal_pos))
            status = 3
            wait_closeDoor(status, goal_pos)
            status = 4
            goal_pub.send_goal(10)
            wait_moving(status, goal_pos)
            goal_pub.send_goal(11)
            wait_moving(status, goal_pos)
            goal_pub.send_goal(12)
            wait_moving(status, goal_pos)
            goal_pub.send_goal(13)
            wait_moving(status, goal_pos)
            goal_pub.send_goal(9999)
            wait_moving(status, goal_pos)
            status = 1
            rospy.loginfo('Arrived at Home')

        elif goal_pos == 3:  # 3rd_Engineering
            status = 2
            wait_closeDoor(status, goal_pos)
            goal_pub.send_goal(20)
            wait_moving(status, goal_pos)
            goal_pub.send_goal(21)
            wait_moving(status, goal_pos)
            goal_pub.send_goal(22)
            wait_moving(status, goal_pos)
            goal_pub.send_goal(23)
            wait_moving(status, goal_pos)
            goal_pub.send_goal(24)
            wait_moving(status, goal_pos)
            rospy.loginfo('Arrived at Goal ' + str(goal_pos))
            status = 3
            wait_closeDoor(status, goal_pos)
            status = 4
            goal_pub.send_goal(30)
            wait_moving(status, goal_pos)
            goal_pub.send_goal(31)
            wait_moving(status, goal_pos)
            goal_pub.send_goal(32)
            wait_moving(status, goal_pos)
            goal_pub.send_goal(33)
            wait_moving(status, goal_pos)
            goal_pub.send_goal(34)
            wait_moving(status, goal_pos)
            goal_pub.send_goal(9999)
            wait_moving(status, goal_pos)
            status = 1
            rospy.loginfo('Arrived at Home')
        
        elif goal_pos == 5:  # 5th_Engineering
            status = 2
            wait_closeDoor(status, goal_pos)
            goal_pub.send_goal(40)
            wait_moving(status, goal_pos)
            goal_pub.send_goal(41)
            wait_moving(status, goal_pos)
            goal_pub.send_goal(42)
            wait_moving(status, goal_pos)
            rospy.loginfo('Arrived at Goal ' + str(goal_pos))
            status = 3
            wait_closeDoor(status, goal_pos)
            status = 4
            goal_pub.send_goal(50)
            wait_moving(status, goal_pos)
            goal_pub.send_goal(51)
            wait_moving(status, goal_pos)
            goal_pub.send_goal(9999)
            wait_moving(status, goal_pos)
            status = 1
            rospy.loginfo('Arrived at Home')
        ########## Arrive at Goal ##########
        


if __name__ == '__main__':
    try:
        dalsu_main()
    except rospy.ROSInterruptException:
        pass