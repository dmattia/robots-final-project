#!/usr/bin/env python

import rospy
import requests
import json
from geometry_msgs.msg import Twist

def web_pub():
    pub_twist = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=10)
    rospy.init_node('web_pub', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    #twist = Twist()
    #twist.linear.x = 0.3
    #twist.angular.z = 0.3
    players = {
        'Blue Player': 0,
	'Green Player': 0,
	'Red Player': 0,
	'Yellow Player': 0
    }

    while not rospy.is_shutdown():
	r = requests.get('https://ros.firebaseio.com/.json')
	data = r.json()
	#rospy.loginfo("New Request with status: " + str(r.status_code))
	update_made = False
	if r.status_code == 200:
	    # If any values in @data do not match values in @players,
	    # then an update must have occurred
	    for player_name in players:
		player_score = data[player_name]
		if player_score != players[player_name]:
			#rospy.loginfo(str(player_score) + " does not equal " + str(players[player_name]))
			update_made = True
		players[player_name] = player_score
	if update_made:
            rospy.loginfo("Update made")
	    for player_name in players:
		rospy.loginfo("    " + player_name + ": " + str(players[player_name]))

        #pub_twist.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        web_pub()
    except rospy.ROSInterruptException:
        pass
