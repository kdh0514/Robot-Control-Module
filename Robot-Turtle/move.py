#!/usr/bin/env python
import roslib
roslib.load_manifest('turtlebot_actions')

import rospy

import os
import sys
import time
import math
from turtlebot_actions.msg import *
from actionlib_msgs.msg import *
from turtlebot_actions.msg import move_to
from sensor_msgs.msg import Range
from turtlebot_actions.msg import boardstate
from turtlebot_actions.msg import robotstate


import actionlib

'''
  Very simple move action test - commands the robot to turn 45 degrees and travel 0.5 metres forward.
'''
class ActionClient():

  seq_id = 10
  sen1 = 1
  sen2 = 1
  sen3 = 1
  sen4 = 1

  def move_to_point1(self, data):
    self.seq_id = data.point_id
    rospy.loginfo("MOVE to Point 1")
  def move_to_point2(self, data):
    self.seq_id = data.point_id
    rospy.loginfo("MOVE to Point 2")
  def move_to_point3(self, data):
    self.seq_id = data.point_id
    rospy.loginfo("MOVE to Point 3")
  def move_to_point4(self, data):
    self.seq_id = data.point_id
    rospy.loginfo("MOVE to Point 4")
  def move_to_point5(self, data):
    self.seq_id = data.point_id
    rospy.loginfo("MOVE to Point 0")

  def get_range1(self, sensor):
    self.ir_range1 = sensor.range
    #rospy.loginfo(self.ir_range1)
    if self.ir_range1 > 10:
      self.sen1 = 0
    else:
      self.sen1 = 1

  def get_range2(self, sensor):
    self.ir_range2 = sensor.range 
    #rospy.loginfo(self.ir_range2)
    if self.ir_range2 > 10:
      self.sen2 = 0
    else:
      self.sen2 = 1
  
  def get_range3(self, sensor):
    self.ir_range3 = sensor.range
    #rospy.loginfo(self.ir_range3)  
    if self.ir_range3 > 10:
      self.sen3 = 0
    else:
      self.sen3 = 1
   
  def get_range4(self, sensor):
    self.ir_range4 = sensor.range 
    #rospy.loginfo(self.ir_range4)
    if self.ir_range4 > 10:
      self.sen4 = 0
    else:
      self.sen4 = 1

  #def sensor1_callback(self, sensor):
    #self.ir_sensor = sensor.range
    #msg = pub_topic_name
   # if (self.ir_sensor >= 10):  
      #msg.board_state_id = 2
     #msg.sensor1 = 0
     # msg.sensor2 = 1 
     # msg.sensor3 = 1
    #  msg.sensor4 = 1
   # else:
     # msg.board_state_id = 
    # msg.sensor1 = 0
   #   msg.sensor2 = 1 
  #    msg.sensor3 = 1
 #     msg.sensor4 = 1

  def __init__(self):

    rospy.init_node("test_move_action_client")
    msg_pub = rospy.Publisher( '/board_state', boardstate, queue_size = 1)
    msg2_pub = rospy.Publisher( '/robot_state', robotstate, queue_size = 1)
    
    State = 1
    Position = 1
     
    b_msg = boardstate()
    b_msg.board_state_id = 0
    b_msg.sensor1 = 0
    b_msg.sensor2 = 0
    b_msg.sensor3 = 0
    b_msg.sensor4 = 0

    msg = robotstate() 
   
    # Construct action ac
    rospy.loginfo("Starting action client...")
    action_client = actionlib.SimpleActionClient('turtlebot_move', TurtlebotMoveAction)
    action_client.wait_for_server()
    rospy.loginfo("Action client connected to action server.")
    
    self.topic1_sub = rospy.Subscriber('/move_to_p1', move_to, self.move_to_point1, queue_size = 1)
    self.topic1_sub = rospy.Subscriber('/move_to_p2', move_to, self.move_to_point2, queue_size = 1)
    self.topic1_sub = rospy.Subscriber('/move_to_p3', move_to, self.move_to_point3, queue_size = 1)
    self.topic1_sub = rospy.Subscriber('/move_to_p4', move_to, self.move_to_point4, queue_size = 1)  
    self.topic1_sub = rospy.Subscriber('/move_to_p0', move_to, self.move_to_point5, queue_size = 1)

    self.range_sub = rospy.Subscriber('/range_data_S1', Range, self.get_range1, queue_size = 1)
    self.range_sub2 = rospy.Subscriber('/range_data_S2', Range, self.get_range2, queue_size = 1)
    self.range_sub3 = rospy.Subscriber('/range_data_S3', Range, self.get_range3, queue_size = 1)
    self.range_sub4 = rospy.Subscriber('/range_data_S4', Range, self.get_range4, queue_size = 1)
    
    rospy.loginfo("After Subscribing... ")
    
    State = 1
    Position = 0
    msg.state_id = State
    msg.position_id = Position 

    #msg2_pub.publish(msg)
    b_msg.board_state_id = 1
    b_msg.sensor1 = self.sen1
    b_msg.sensor2 = self.sen2
    b_msg.sensor3 = self.sen3
    b_msg.sensor4 = self.sen4
    #msg_pub.publish(b_msg)
    
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
     
      msg2_pub.publish(msg)
      rospy.loginfo(msg)
      rospy.loginfo("publishing robot msg...")
      #msg_pub.publish(b_msg)
      rospy.loginfo("this is publishing board msg...")
      rospy.loginfo(self.seq_id)
      

      
      buf = 0 
      if (self.seq_id == 1):
        rospy.loginfo("This is outside the loop...")
        rospy.loginfo(self.seq_id)
        State = 2
        Position = 0 
        msg.state_id = State
        msg.position_id = Position
        msg2_pub.publish(msg)
        rospy.loginfo(State)

        if (State == 2):
          rospy.loginfo("test point ... !! ")
          action_goal1 = TurtlebotMoveGoal()
          action_goal1.forward_distance = 1 # metres
          #action_goal.turn_distance = -math.pi/2.0 + 0.25

          if action_client.send_goal_and_wait(action_goal1, rospy.Duration(100.0)) == GoalStatus.SUCCEEDED:
            rospy.loginfo('Call to action server succeeded')
            State = 4
            Position = 1 
            msg.state_id = State
            msg.position_id = Position
            msg2_pub.publish(msg)
            self.seq_id = 10
            rospy.loginfo(self.seq_id)

            while (buf == 0):

              rospy.loginfo("test point sensor1 ... !! ")
              rospy.loginfo(self.sen1)
              if(self.sen1 == 0):
                
                b_msg.board_state_id = 2
                b_msg.sensor1 = self.sen1
                b_msg.sensor2 = self.sen2
                b_msg.sensor3 = self.sen3
                b_msg.sensor4 = self.sen4
                msg_pub.publish(b_msg)
 
                buf = 1
       
              if(self.sen2 == 0):
   
                b_msg.board_state_id = 2
                b_msg.sensor1 = self.sen1
                b_msg.sensor2 = self.sen2
                b_msg.sensor3 = self.sen3
                b_msg.sensor4 = self.sen4
                msg_pub.publish(b_msg)
                buf = 1
  
              if(self.sen3 == 0):
                b_msg.board_state_id = 2
                b_msg.sensor1 = self.sen1
                b_msg.sensor2 = self.sen2
                b_msg.sensor3 = self.sen3
                b_msg.sensor4 = self.sen4
                msg_pub.publish(b_msg)
                buf = 1
 
              if(self.sen4 == 0):
                b_msg.board_state_id = 2
                b_msg.sensor1 = self.sen1
                b_msg.sensor2 = self.sen2
                b_msg.sensor3 = self.sen3
                b_msg.sensor4 = self.sen4
                msg_pub.publish(b_msg)
                buf = 1

      elif (self.seq_id == 2):
         
        State = 2
        Position = 1
        msg.state_id = State
        msg.position_id = Position
        msg2_pub.publish(msg)
       
        if (State == 2):
          action_goal2 = TurtlebotMoveGoal()
          action_goal2.forward_distance = 1.4 # metres
          #action_goal.turn_distance = -math.pi/2.0 + 0.25
          if action_client.send_goal_and_wait(action_goal2, rospy.Duration(100.0)) == GoalStatus.SUCCEEDED:
            rospy.loginfo('Call to action server succeeded')
            State = 4
            Position = 2 
            msg.state_id = State
            msg.position_id = Position
            msg2_pub.publish(msg)
            self.seq_id = 10
          

            while (buf == 0):
      
              rospy.loginfo(self.sen1)
              rospy.loginfo(self.sen2)
              rospy.loginfo('Checking sensor values for point 3...!!!')
              
              if self.sen1 == 0 and self.sen2 == 0:
   
                b_msg.board_state_id = 2
                b_msg.sensor1 = self.sen1
                b_msg.sensor2 = self.sen2
                b_msg.sensor3 = self.sen3
                b_msg.sensor4 = self.sen4
                msg_pub.publish(b_msg)
           
 
                buf = 1
       
              if self.sen1 == 0 and self.sen3 == 0:
   
                b_msg.board_state_id = 2
                b_msg.sensor1 = self.sen1
                b_msg.sensor2 = self.sen2
                b_msg.sensor3 = self.sen3
                b_msg.sensor4 = self.sen4
                msg_pub.publish(b_msg)
              
                buf = 1
    
              if self.sen1 == 0 and self.sen4 == 0:
     
                b_msg.board_state_id = 2
                b_msg.sensor1 = self.sen1
                b_msg.sensor2 = self.sen2
                b_msg.sensor3 = self.sen3
                b_msg.sensor4 = self.sen4
                msg_pub.publish(b_msg)
                buf = 1
 
              if self.sen2 == 0 and self.sen3 == 0:
              
                b_msg.board_state_id = 2
                b_msg.sensor1 = self.sen1
                b_msg.sensor2 = self.sen2
                b_msg.sensor3 = self.sen3
                b_msg.sensor4 = self.sen4
                msg_pub.publish(b_msg)
                buf = 1

              if self.sen2 == 0 and self.sen4 == 0:
                
                b_msg.board_state_id = 2
                b_msg.sensor1 = self.sen1
                b_msg.sensor2 = self.sen2
                b_msg.sensor3 = self.sen3
                b_msg.sensor4 = self.sen4
                msg_pub.publish(b_msg)
                buf = 1

              if self.sen3 == 0 and self.sen4 == 0:
          
    
                b_msg.board_state_id = 2
                b_msg.sensor1 = self.sen1
                b_msg.sensor2 = self.sen2
                b_msg.sensor3 = self.sen3
                b_msg.sensor4 = self.sen4
                msg_pub.publish(b_msg)
                buf = 1
  
    
      elif (self.seq_id == 3):
 
        State = 2
        Position = 2
        msg.state_id = State
        msg.position_id = Position
        msg2_pub.publish(msg)
 
        if (State == 2):
 
          action_goal3 = TurtlebotMoveGoal()
          action_goal3.forward_distance = 0.35 # metres
          action_goal3.turn_distance = -math.pi/2.0 + 0.3
          if action_client.send_goal_and_wait(action_goal3, rospy.Duration(100.0)) == GoalStatus.SUCCEEDED:
            rospy.loginfo('Call to action server succeeded')
            rospy.sleep(1)

            self.seq_id = 10
           
            State = 4
            Position = 3
            msg.state_id = State
            msg.position_id = Position
            msg2_pub.publish(msg)
      
       
        
            while (buf == 0):

              if self.sen1 == 0 and self.sen2 == 0 and self.sen3 == 0:
                
   
                b_msg.board_state_id = 2
                b_msg.sensor1 = self.sen1
                b_msg.sensor2 = self.sen2
                b_msg.sensor3 = self.sen3
                b_msg.sensor4 = self.sen4
                msg_pub.publish(b_msg)
 
                buf = 1
       
              if self.sen1 == 0 and self.sen3 == 0 and self.sen4 == 0:
              
                b_msg.board_state_id = 2
                b_msg.sensor1 = self.sen1
                b_msg.sensor2 = self.sen2
                b_msg.sensor3 = self.sen3
                b_msg.sensor4 = self.sen4
                msg_pub.publish(b_msg)
                buf = 1
     
              if self.sen1 == 0 and self.sen2 == 0 and self.sen4 == 0:
            
   
                b_msg.board_state_id = 2
                b_msg.sensor1 = self.sen1
                b_msg.sensor2 = self.sen2
                b_msg.sensor3 = self.sen3
                b_msg.sensor4 = self.sen4
                msg_pub.publish(b_msg)
                buf = 1
 
              if self.sen2 == 0 and self.sen3 == 0 and self.sen4 == 0:
           
   
                b_msg.board_state_id = 2
                b_msg.sensor1 = self.sen1
                b_msg.sensor2 = self.sen2
                b_msg.sensor3 = self.sen3
                b_msg.sensor4 = self.sen4
                msg_pub.publish(b_msg)

                buf = 1      
        
      elif (self.seq_id == 4): 
       

        State = 2
        Position = 3
        msg.state_id = State
        msg.position_id = Position
        msg2_pub.publish(msg)
     
        if (State == 2):
          action_goal6 = TurtlebotMoveGoal()
          action_goal6.forward_distance = 1.4 # metres
          action_goal6.turn_distance = -math.pi/2.0 + 0.25
          if action_client.send_goal_and_wait(action_goal6, rospy.Duration(100.0)) == GoalStatus.SUCCEEDED:
            rospy.loginfo('Call to action server succeeded')
            
            State = 4
            Position = 4
            msg.state_id = State
            msg.position_id = Position
            msg2_pub.publish(msg)
            self.seq_id = 10

            while (buf == 0):

              if self.sen1 == 0 and self.sen2 == 0 and self.sen3 == 0 and self.sen4 == 0:
                State = 3
                Position = 4 
                msg.state_id = State
                msg.position_id = Position
                msg2_pub.publish(msg)
   
                b_msg.board_state_id = 0
                b_msg.sensor1 = self.sen1
                b_msg.sensor2 = self.sen2
                b_msg.sensor3 = self.sen3
                b_msg.sensor4 = self.sen4
                msg_pub.publish(b_msg)
 
                buf = 1
       
             
      elif (self.seq_id == 0):
         
        State = 1
        Position = 4
        msg.state_id = State
        msg.position_id = Position
        msg2_pub.publish(msg)
        if (State == 1):
          action_goal7 = TurtlebotMoveGoal()
          action_goal7.forward_distance = 0.85 # metres
          #action_goal.turn_distance = -math.pi/2.0 + 0.25
          if action_client.send_goal_and_wait(action_goal7, rospy.Duration(50.0), rospy.Duration(50.0)) == GoalStatus.SUCCEEDED:
            rospy.loginfo('Call to action server succeeded')
            action_goal8 = TurtlebotMoveGoal()
            action_goal8.forward_distance = 0.4 # metres
            action_goal8.turn_distance = -math.pi/2.0 + 0.4
            action_client.send_goal_and_wait(action_goal8, rospy.Duration(50.0), rospy.Duration(50.0)) == GoalStatus.SUCCEEDED

            State = 1
            Position = 0
            msg.state_id = State
            msg.position_id = Position
            msg2_pub.publish(msg)

            b_msg.board_state_id = 0
            b_msg.sensor1 = self.sen1
            b_msg.sensor2 = self.sen2
            b_msg.sensor3 = self.sen3
            b_msg.sensor4 = self.sen4
            msg_pub.publish(b_msg)
            self.seq_id = 10
            
      rospy.sleep(0.5)
    #rospy.spin()
    ###while choice != 'q':
      ###choice = self.choose()
      ###if (choice == 1):
        ###rospy.loginfo("This is inside the loop...")
   	###action_goal = TurtlebotMoveGoal()
        ###action_goal.forward_distance = 0.01 # metres
        ###action_goal.turn_distance = -math.pi/2.0 + 0.25
        ###if action_client.send_goal_and_wait(action_goal, rospy.Duration(100.0), rospy.Duration(100.0)) == GoalStatus.SUCCEEDED:
          ###rospy.loginfo('Call to action server succeeded')
        ###else:
          ###rospy.logerr('Call to action server failed')
		
    # Call the action
    #rospy.loginfo("Calling the action server...")
    #for i in range(0, 4) :
	##action_goal = TurtlebotMoveGoal()
	##action_goal.forward_distance = 1 # metres
        #action_goal.turn_distance = -math.pi/2.0 + 0.25
	
        ##if action_client.send_goal_and_wait(action_goal, rospy.Duration(100.0), rospy.Duration(100.0)) == GoalStatus.SUCCEEDED:
          ##rospy.loginfo('Call to action server succeeded')
        ##else:
          ##rospy.logerr('Call to action server failed')
    # action_goal = TurtlebotMoveGoal()
    # action_goal.turn_distance = -math.pi/2.0
    # action_goal.forward_distance = 1 # metres
	##break
    # if action_client.send_goal_and_wait(action_goal, rospy.Duration(50.0), rospy.Duration(50.0)) == GoalStatus.SUCCEEDED:
    #   rospy.loginfo('Call to action server succeeded')
    # else:
    #   rospy.logerr('Call to action server failed')

if __name__ == "__main__":
  ActionClient()
