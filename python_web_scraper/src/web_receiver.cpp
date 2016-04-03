#include <python_web_scraper/Vector4.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <ros/ros.h>
#include <cstdarg>
#include <string>
#include <ctime>

// Global data structures

enum player {
	BLUE,
	GREEN,
	RED,
	YELLOW
};
enum difficulty {
	EASY,
	MEDIUM,
	HARD
};
typedef python_web_scraper::Vector4 score_set;

// Global variables
bool should_print;
bool game_won;
enum difficulty mode = EASY;
score_set previousScores = score_set();
enum player winner;
time_t lastScored;

void print(const char *format...) {
	if (should_print) {
		va_list argptr;
    	va_start(argptr, format);
   	vfprintf(stdout, format, argptr);
    	va_end(argptr);
		printf("\n");
	}
}

ros::Publisher sound_pub;
void playSong(const std::string str) {
	std_msgs::String sound_str;
	sound_str.data = str;
	sound_pub.publish(sound_str);
}

bool isaWinner(const score_set scores) {
	if (scores.blue == 5) {
		winner = BLUE;
		return true;
	} else if (scores.red == 5) {
		winner = RED;
		return true;
	} else if (scores.green == 5) {
		winner = GREEN;
		return true;
	} else if (scores.yellow == 5) {
		winner = YELLOW;
		return true;
	} else {
		return false;
	}
} 

void difficultyCallback(const std_msgs::Int8 difficulty) {
	mode = (enum difficulty)(difficulty.data);
	print("New difficulty is: %d", difficulty.data);
}

void scoreCallback(const score_set updatedScores) {
	print("Received an updated score");

	print("	 Blue score: %d", updatedScores.blue);
	print("	 Green score: %d", updatedScores.green);
	print("	 Red score: %d", updatedScores.red);
	print("	 Yellow score: %d", updatedScores.yellow);

	if (updatedScores.blue > previousScores.blue) {
		playSong("Ravenclaw Scored. yay");
	}
	if (updatedScores.green > previousScores.green) {
		playSong("Slitherin Scored. yay");
	}
	if (updatedScores.red > previousScores.red) {
		playSong("Griffindoor Scored. yay");
	}
	if (updatedScores.yellow > previousScores.yellow) {
		playSong("Hufflepuff Scored. yay");
	}
	if (isaWinner(updatedScores)) {
		game_won = true;
	}

	previousScores = updatedScores;
}

void celebrate(enum player winningPlayer) {
	switch(winningPlayer) {
		case RED:
			playSong("Griffindoor won");
			break;
		case BLUE:
			playSong("Ravenclaw won");
			break;
		case GREEN:
			playSong("Slitherin won");
			break;
		case YELLOW:
			playSong("Hufflepuff won");
			break;
	}
}

void endWithoutWinner() {
	print("Time expired before anyone could win");
	playSong("Time expired before anyone could win");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "web_receiver");
	ros::NodeHandle n;

	ros::Subscriber difficultySubscriber = n.subscribe("/web_pub/difficulty", 10, difficultyCallback);
	ros::Subscriber scoreSubscriber = n.subscribe("/web_pub/scores", 10, scoreCallback);
	ros::Publisher velocityPublisher = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1000);
	sound_pub = n.advertise<std_msgs::String>("sound_to_play", 1000);

	geometry_msgs::Twist twist;
	time_t startTime = time(NULL);
	
	/*
	std::string print_verbose;
	bool got_print_verbose = ros::param::get("print_verbose", print_verbose);
	if(!got_print_verbose) {
		ROS_ERROR("Could not find parameter @print_verbose");
		return 1;
	} else {
		if(print_verbose == "1") {
			should_print = true;
		} else {
			should_print = false;
		}	
	}
	*/
	should_print = true;
	lastScored = time(NULL);

	ros::Rate loop_rate(10);
	while(ros::ok())
	{
		time_t timeElapsedSinceProgramBegan = time(NULL) - startTime;
		time_t timeElapsedSinceLastScore = time(NULL) - lastScored;

		switch(mode) {
			case EASY:
				twist.linear.x = 0.15;
				twist.angular.z = 0.4;
				break;
			case MEDIUM:
				twist.linear.x = 0.15;
				twist.angular.z = 0.4;
				break;
			case HARD:
				twist.linear.x = 0.15;
				twist.angular.z = 0.4;
				break;
		}
		velocityPublisher.publish(twist);
		ros::spinOnce();
		loop_rate.sleep();

		if(game_won) {
			celebrate(winner);
			break;
		}
		if(timeElapsedSinceProgramBegan > 10) {
			endWithoutWinner();
			break;
		}
	} 
	return 0;
}
