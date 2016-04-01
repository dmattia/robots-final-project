#include <python_web_scraper/Vector4.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <cstdarg>
#include <string>
#include <ctime>

// Global data structures

enum difficulty {
	EASY,
	MEDIUM,
	HARD
};
typedef python_web_scraper::Vector4 score_set;

// Global variables
bool should_print;
enum difficulty mode = EASY;

void print(const char *format...) {
	if (should_print) {
		va_list argptr;
    	va_start(argptr, format);
   	vfprintf(stdout, format, argptr);
    	va_end(argptr);
		printf("\n");
	}
}

void playSong(const std::string filename) {
	print("Playing sound file: %s", filename.c_str());
}

score_set previousScores = score_set();
void scoreCallback(const score_set updatedScores) {
	print("Received an updated score");

	print("	 Blue score: %d", updatedScores.blue);
	print("	 Green score: %d", updatedScores.green);
	print("	 Red score: %d", updatedScores.red);
	print("	 Yellow score: %d", updatedScores.yellow);

	if (updatedScores.blue > previousScores.blue) {
		playSong("blue.mp3");
	}
	if (updatedScores.green > previousScores.green) {
		playSong("green.mp3");
	}
	if (updatedScores.red > previousScores.red) {
		playSong("red.mp3");
	}
	if (updatedScores.yellow > previousScores.yellow) {
		playSong("yellow.mp3");
	}

	previousScores = updatedScores;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "web_receiver");
	ros::NodeHandle n;

	ros::Subscriber scoreSubscriber = n.subscribe("/web_pub/scores", 10, scoreCallback);
	ros::Publisher velocityPublisher = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1000);

	geometry_msgs::Twist twist;
	
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

	ros::Rate loop_rate(10);
	while(ros::ok())
	{
		switch(mode) {
			case EASY:
				//twist.linear.x = 0.15;
				//twist.angular.z = 0.4;
				break;
			case MEDIUM:
				break;
			case HARD:
				break;
		}
		velocityPublisher.publish(twist);
		ros::spinOnce();
		loop_rate.sleep();
	} 
	return 0;
}
