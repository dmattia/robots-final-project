#!/usr/bin/env python

import rospy
import requests
import json
from python_web_scraper.msg import Vector4

def web_pub():
    score_publisher = rospy.Publisher('/web_pub/scores', Vector4, queue_size=10)
    rospy.init_node('web_pub', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    players = {
        'Blue Player': 0,
	'Green Player': 0,
	'Red Player': 0,
	'Yellow Player': 0
    }

    while not rospy.is_shutdown():
	r = requests.get('https://ros.firebaseio.com/.json')
	data = r.json()
	update_made = False
	if r.status_code == 200:
	    # If any values in @data do not match values in @players,
	    # then an update must have occurred
	    for player_name in players:
		player_score = data[player_name]
		if player_score != players[player_name]:
			update_made = True
		players[player_name] = player_score
	if update_made:
            rospy.loginfo("Update made")
	    for player_name in players:
		rospy.loginfo("    " + player_name + ": " + str(players[player_name]))

	    scores = Vector4()
	    scores.blue = players['Blue Player']
	    scores.green = players['Green Player']
	    scores.red = players['Red Player']
	    scores.yellow = players['Yellow Player']
	    score_publisher.publish(scores)

        rate.sleep()

if __name__ == '__main__':
    try:
        web_pub()
    except rospy.ROSInterruptException:
        pass
    except:
			web_pub()
