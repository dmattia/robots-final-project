#include <python_web_scraper/Vector4.h>
#include <geometry_msgs/Twist.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
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
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// Global variables
bool should_print;
bool game_won;
enum difficulty mode = EASY;
score_set previousScores = score_set();
enum player winner;
time_t lastScored;

bool hasSetFirstPoint = false;
pcl::PointXYZ closestPoint;

void pclCallback(const PointCloud::ConstPtr& cloud)
{
   float min_z = 1e6;
   BOOST_FOREACH (const pcl::PointXYZ& pt, cloud->points)
   {
		if (pt.z < min_z) {
			hasSetFirstPoint = true;
			closestPoint = pt;
			min_z = pt.z;
		}
   }
}

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
	if (scores.blue == 10) {
		winner = BLUE;
		return true;
	} else if (scores.red == 10) {
		winner = RED;
		return true;
	} else if (scores.green == 10) {
		winner = GREEN;
		return true;
	} else if (scores.yellow == 10) {
		winner = YELLOW;
		return true;
	} else {
		return false;
	}
} 

enum player whoWon(const score_set scores) {
	int playerScores[] = {scores.blue, scores.green, scores.red, scores.yellow};
	int maxScoreIndex = 0;
	for(int i=1; i < 4; ++i) {
		if (playerScores[i] > maxScoreIndex) {
			maxScoreIndex = i;
		}
	}
	return (enum player)(maxScoreIndex);
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
		playSong("Ten Points to Ravenclaw!");
	}
	if (updatedScores.green > previousScores.green) {
		playSong("Ten Points to Slitherin!");
	}
	if (updatedScores.red > previousScores.red) {
		playSong("Ten Points to Griffindoor!");
	}
	if (updatedScores.yellow > previousScores.yellow) {
		playSong("Ten Points to Hufflepuff!");
	}
	if (isaWinner(updatedScores)) {
		game_won = true;
	}

	previousScores = updatedScores;
}

void celebrate(enum player winningPlayer) {
	switch(winningPlayer) {
		case RED:
			playSong("Griffindoor wins the house cup. yay!");
			break;
		case BLUE:
			playSong("Ravenclaw wins the house cup. yay!");
			break;
		case GREEN:
			playSong("Slitherin wins the house cup. yay!");
			break;
		case YELLOW:
			playSong("Hufflepuff wins the house cup. yay!");
			break;
	}
}

// Returns one of 1000 values in [@min, @max]
float randomValueInRange(float min, float max) {
	int randValue = rand() % 1000;
	float range = max - min;
	float step = range / 1000;
	return min + (step * randValue);
}

void keepInRange(double &value, double min, double max) {
	if (value < min) value = min;
	if (value > max) value = max;
}

void dance(ros::Publisher velocityPublisher, ros::Rate loop_rate) {
	geometry_msgs::Twist twist;
	for(int i=0; i < 10; ++i) {
		twist.linear.x = 0;
		twist.angular.z = 9.0;
		velocityPublisher.publish(twist);
		ros::spinOnce();
		loop_rate.sleep();
	}	
	for(int i=0; i < 10; ++i) {
		twist.linear.x = 0;
		twist.angular.z = -9.0;
		velocityPublisher.publish(twist);
		ros::spinOnce();
		loop_rate.sleep();
	}	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "web_receiver");
	ros::NodeHandle n;

	ros::Subscriber difficultySubscriber = n.subscribe("/web_pub/difficulty", 10, difficultyCallback);
	ros::Subscriber scoreSubscriber = n.subscribe("/web_pub/scores", 10, scoreCallback);
  	ros::Subscriber pointCloudSubscriber = n.subscribe<PointCloud>("/camera/depth/points", 1, pclCallback);

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
	int changeLimit = 20;
	int count = 0;
	double beforeObjectAvoidance;
	srand (time(NULL));
	bool isMedium = false;
	bool objectIsClose = false;
	bool shouldUseOldSpeed = false;
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
				if (!isMedium) {
					twist.linear.x = 0.7;
					twist.angular.z = 1.0;
					isMedium = true;
				}
				if(closestPoint.z < 0.48 && hasSetFirstPoint) {
					if(!shouldUseOldSpeed) {
						beforeObjectAvoidance = twist.linear.x;
						twist.linear.x = 0.0;
					}
					shouldUseOldSpeed = true;
				} else {
					if(shouldUseOldSpeed) {
						twist.linear.x = beforeObjectAvoidance;
						shouldUseOldSpeed = false;
					}
					twist.linear.x -= 0.001;
					keepInRange(twist.linear.x, 0.2, 0.7);
					twist.angular.z = (1.0 * twist.linear.x) / 0.7;
				}
				break;
			case HARD:
				isMedium = false;
				if (!(count++ % changeLimit) && !objectIsClose) {
					twist.linear.x = randomValueInRange(0.2, 0.5);
					twist.angular.z = randomValueInRange(-1.5, 1.5);
					count++;
				}
				if(closestPoint.z < 0.48 && hasSetFirstPoint) {
					if(!objectIsClose) {
						if(closestPoint.x < 0) {
							twist.linear.x = randomValueInRange(-0.1, 0.1);
							twist.angular.z = randomValueInRange(-1.5, -0.5);
						} else {
							twist.linear.x = randomValueInRange(-0.1, 0.1);
							twist.angular.z = randomValueInRange(0.5, 1.5);
						}
						objectIsClose = true;
					}
				} else {
					objectIsClose = false;
				}
				break;
		}
		velocityPublisher.publish(twist);
		ros::spinOnce();
		loop_rate.sleep();

		if(game_won) {
			celebrate(winner);
			dance(velocityPublisher, loop_rate);
			break;
		}
		if(timeElapsedSinceProgramBegan > 40) {
			celebrate(whoWon(previousScores));
			dance(velocityPublisher, loop_rate);
			break;
		}
	} 
	return 0;
}
