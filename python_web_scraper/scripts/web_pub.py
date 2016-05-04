#!/usr/bin/env python

import rospy
import requests
import json
from python_web_scraper.msg import Vector4
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import Int8, String
from geometry_msgs.msg import Twist
from enum import Enum

class Difficulty(Enum):
    EASY = 0
    MEDIUM = 1
    HARD = 2

def playSound(data):
	""" Function Name: playSound
		Parameters: data (A string to be converted to speech)
		Returns: None
	
		Description: Sends @data to the sound_play node to be spoken
	"""
    soundhandle = SoundClient()
    soundhandle.say(data.data)

lastTwist = Twist()
def updateMovement(data):
	# FOR iOS TELEOP, NOT FOR PROJECT
	""" Function Name: updateMovement
		Parameters: data (Twist object containing data for teleop)
		Returns: None
	
		Description: Checks if any data has changed since the last time this function was
			called.  Publishes the data to the C++ node if it has
	"""
    if lastTwist.linear.x != data.linear.x or lastTwist.angular.z != data.angular.z:
    	teleop_publisher = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=10)
    	teleop_publisher.publish(data)
    lastTwist.linear.x = data.linear.x
    lastTwist.angular.z = data.angular.z

def web_pub():
	""" Function Name: web_pub
		Parameters: none
		Returns: None
	
		Description: Main function for the python node.
			Sets up publishers and subscribers.
			Scrapes the firebase database for updates in the iOS app.
	"""
    rospy.init_node('web_pub', anonymous=True)

    score_publisher = rospy.Publisher('/web_pub/scores', Vector4, queue_size=10)
    difficulty_publisher = rospy.Publisher('/web_pub/difficulty', Int8, queue_size=3)
    motion_publisher = rospy.Publisher('ios_teleop', Twist, queue_size=10)

    rospy.Subscriber("sound_to_play", String, playSound)
    rospy.Subscriber("ios_teleop", Twist, updateMovement)

    previousDifficulty = Difficulty.EASY
    players = {
	'Blue Player': 0,
	'Green Player': 0,
	'Red Player': 0,
	'Yellow Player': 0
    }

    motion = Twist()

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
	r = requests.get('https://ros.firebaseio.com/.json')
	data = r.json()

	update_made = False
	if r.status_code == 200:
	    # If any values in @scoreData do not match values in @players,
	    # then an update must have occurred
	    scoreData = data["scores"]
	    difficultyData = data["difficulty"]

	    motionData = data["speed"]
	    motion.linear.x = 0.3 * motionData["linear"]
	    motion.angular.z = -2 * motionData["angular"]
	    motion_publisher.publish(motion)

	    for player_name in players:
		player_score = scoreData[player_name]
		if player_score != players[player_name]:
			update_made = True
		players[player_name] = player_score
	    if int(difficultyData) != previousDifficulty.value:
		previousDifficulty = Difficulty(int(difficultyData))
		mode = Int8()
		mode.data = int(previousDifficulty.value)
		difficulty_publisher.publish(mode)
	if update_made:
	    rospy.loginfo("Update made")
	    for player_name in players:
		rospy.loginfo("    " + player_name + ": " + str(players[player_name]))

	    scores = Vector4()
	    scores.blue = players['Blue Player']
	    scores.green = players['Green Player']
	    scores.red = players['Red Player']
	    scores.yellow = players['Yellow Player']
	    score_publisher.publish(scores) # to C++ node

	rate.sleep()

if __name__ == '__main__':
    try:
	web_pub()
    except rospy.ROSInterruptException:
	pass
    except:
	pass
