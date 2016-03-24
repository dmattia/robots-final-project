# robots-final-project

This is part of my final project for Autonomous Mobile Robots. Our robot uses a hybrid controller to play games meant for kids.

## Ring Toss

For this game, our robot has a pole on top that kids can toss plastic rings onto. Score is kept track of through another project of mine, ros-scoreboard, that I developed to work in tandem with this project.  The robots motion and actions are controlled by multiple sensor callbacks in addition to a final callback that is called asynchronously when the database is updated for the scoreboard.

## Project Setup

The main data for this project (and ros-scoreboard) comes from a single firebase database.  This database holds json values for four players scores, such as:

    {
      "Blue Player": 4,
      "Red Player": 3,
      "Green Player": 2,
      "Yellow Player": 1
    }

The scoreboard, found at https://ros.firebaseapp.com/, presents the score live. The python node in this project uses get requests on https://ros.firebaseapp.com/.json repeatedly to check for updates.  When it finds some value has changed, it will publish a message of type Vector4 (custom) to a topic that the c++ node subscribes to.  The robot can then react to the score updates by playing a sound, changing speed, dancing if someone reached 5 points, etc.

