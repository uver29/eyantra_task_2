#!/usr/bin/env python3

import rospy
import tf
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import math

# waypoints = [{'x': -9.1, 'y': -1.2, 'w': },
#             {'x': 10.7, 'y': 10.5, 'w': },
#             {'x': 12.6, 'y': -1.9, 'w': },
#             {'x': 18.2, 'y': -1.4, 'w': },
#             {'x': -2.0, 'y': 4.0, 'w': }]
waypoints = [{'x': -9.1, 'y': -1.2, 'w': 1.2},
            {'x': 10.7, 'y': 10.5, 'w': 0.7},
            {'x': 12.6, 'y': -1.6, 'w': 0.7},
            {'x': 18.2, 'y': -1.4, 'w': -1.0},
            {'x': -2.0, 'y': 4.0, 'w': 1.0}]
waypoint_index = 0

rospy.init_node('navigate_waypoints_py')
client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
rospy.loginfo("Waiting for move_base action server...")
wait = client.wait_for_server(rospy.Duration(5.0))
if not wait:
    rospy.logerr("Action server not available!")
    rospy.signal_shutdown("Action server not available!")
    exit()
rospy.loginfo("Connected to move base server")
rospy.loginfo("Sending the waypoints ...")

def active_cb():
    rospy.loginfo("Bot moving towards waypoint " + str(waypoint_index + 1))


def feedback_cb(feedback):
    x_feedback = feedback.base_position.pose.position.x
    y_feedback = feedback.base_position.pose.position.y
    w_feedback = feedback.base_position.pose.orientation.w
    rospy.loginfo("Feedback for waypoint " + str(waypoint_index + 1) + ":")
    rospy.loginfo(x_feedback)
    rospy.loginfo(y_feedback)
    rospy.loginfo(w_feedback)


def done_cb(status, result):
    global waypoint_index
    if status == 2:
        rospy.loginfo("Bot moving towards waypoint " + str(waypoint_index + 1) + " received a cancel request after it started executing, completed execution!")

    if status == 3:
        rospy.loginfo("Bot moving towards waypoint " + str(waypoint_index + 1) + " has reached") 
        if waypoint_index < 4:
            waypoint_index += 1
            next_goal = MoveBaseGoal()
            next_goal.target_pose.header.frame_id = "map"
            next_goal.target_pose.header.stamp = rospy.Time.now()
            next_goal.target_pose.pose.position.x = waypoints[waypoint_index]['x']
            next_goal.target_pose.pose.position.y = waypoints[waypoint_index]['y']
            next_goal.target_pose.pose.position.z = 0.0
            quat = tf.transformations.quaternion_from_euler(0, 0, waypoints[waypoint_index]['w'])
            next_goal.target_pose.pose.orientation.x = quat[0]
            next_goal.target_pose.pose.orientation.y = quat[1]
            next_goal.target_pose.pose.orientation.z = quat[2]
            next_goal.target_pose.pose.orientation.w = quat[3]
            rospy.loginfo("Sending waypoint "+str(waypoint_index+1)+" to Action Server")
            client.send_goal(next_goal, done_cb, active_cb, feedback_cb)

        else:
            rospy.loginfo("Final waypoint reached!")
            rospy.signal_shutdown("Final waypoint reached!")
            return

def navigate_waypoints():
    global waypoint_index
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = waypoints[waypoint_index]['x']
    goal.target_pose.pose.position.y = waypoints[waypoint_index]['y']
    goal.target_pose.pose.position.z = 0.0
    quat = tf.transformations.quaternion_from_euler(0, 0, waypoints[waypoint_index]['w'])
    goal.target_pose.pose.orientation.x = quat[0]
    goal.target_pose.pose.orientation.y = quat[1]
    goal.target_pose.pose.orientation.z = quat[2]
    goal.target_pose.pose.orientation.w = quat[3]

    rospy.loginfo("Sending waypoint "+str(waypoint_index+1)+" to Action Server")
    client.send_goal(goal, done_cb, active_cb, feedback_cb)
    rospy.spin()


if __name__ == '__main__':
    try:
        navigate_waypoints()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation interrupted")