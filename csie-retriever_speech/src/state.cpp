#include "ros/ros.h"
#include "std_msgs/String.h"
#include <time.h> 
#include <chrono>
#include <sstream>
#include <string>

enum State {
  MapConstruction, 
  Patrol,
  WaitForReplyWhilePatrol,
  HelpingWhileWaitForPerson,
  HelpPeople,
  GuidingWhileWaitForPerson,
  GuidePeople,
}

enum Place {
  bathroom,
  water_dispenser,
  stairs,
  elevator,
  stop,
  guide
}

typedef struct {
  int x;
  int y;
  int rx;
  int ry;
} P;

typedef struct {
  std::chrono::time_point<std::chrono::system_clock> ts;
  int id;
  int face;
  Place target;
} U;

// public variable;
int current_state = MapConstruction;
int current_target_place = -1;
int timeOutCount = 0;
P current_pose;
P current_target;
U current_wait_user;
U current_user;
map<int, U> lost_user; 
map<Place, P> targetMap;
vector<P> route;
ros::NodeHandle n;
ros::Publisher go_to_target;

void tts(string s) {

}

void find_plan() {
  // find route based on current position
}

/*
  Report constantly
 */

// keep track of current position
void hear_current_pose(const std_msgs::String::ConstPtr& msg) {
  if (current_state == HelpPeople || current_state == GuidePeople 
    || current_state == WaitForReplyWhilePatrol) return;
  current_pose.x = msg.x;
  current_pose.y = msg.y;
  current_pose.rx = msg.rx;
  current_pose.ry = msg.ry;
}

// keep track of person saw
void last_see_people(const std_msgs::String::ConstPtr& msg) {
  current_user.ts = std::chrono::system_clock::now();
  current_user.id = msg.user_num;
  current_user.face = msg.face;
}

/*
  Map Construction callback
 */

// finish map construction
void finish_construction_Callback(const std_msgs::String::ConstPtr& msg) {
  if (current_state != MapConstruction) return;
  if (!strcmp(msg->data.c_str(), "finish construction")) {
    current_state = Patrol;
  } 
}

// find some targets
void hear_find_target_Callback(const std_msgs::String::ConstPtr& msg) {
  if (current_state != MapConstruction) return;
  if (!strcmp(msg->data.c_str(), "bathroom")) {
    targetMap.insert(bathroom, current_pose);
  } else if (!strcmp(msg->data.c_str(), "water_dispenser")) {
    targetMap.insert(water_dispenser, current_pose);
  } else if (!strcmp(msg->data.c_str(), "stairs")) {
    targetMap.insert(stairs, current_pose);
  } else if (!strcmp(msg->data.c_str(), "elevator")) {
    targetMap.insert(elevator, current_pose);
  } else if (!strcmp(msg->data.c_str(), "stop")) {
    targetMap.insert(stop, current_pose);
  }
}

/*
  Patrol Construction callback
 */

void listen_to_people(const std_msgs::String::ConstPtr& msg) {
  if (current_state == WaitForReplyWhilePatrol) {
    if (!strcmp(msg->data.c_str(), "yes")) {
      if (!strcmp(current_wait_user.target, "guide")) {
        current_state = GuidePeople;
      } else {
        current_state = HelpPeople;
      }
      lost_user.erase(current_wait_user.id);
      timeOutCount = 0;
    } else if (!strcmp(msg->data.c_str(), "no")) {
      lost_user.erase(current_wait_user.id);
      current_state = Patrol;
      timeOutCount = 0;
    }
    return;
  }
  if (current_state == Patrol && (std::chrono::system_clock::now() - current_user.ts < 1. 
                                || current_user.face > THRESHOLD)) {
    // check if person is lost
    U user = lost_user.find(current_user.id)
    if (user) {
      current_state = WaitForReplyWhilePatrol;
      current_wait_user = user;
      return;
    } 

    if (!strcmp(msg, "bathroom")) {
      current_target = targetMap.find(bathroom);
      current_state = HelpPeople;
      current_user.target = bathroom;
    } else if (!strcmp(msg, "water_dispenser")) {
      current_target = targetMap.find(water_dispenser);
      current_state = HelpPeople;
      current_user.target = water_dispenser;
    } else if (!strcmp(msg, "stairs")) {
      current_target = targetMap.find(stairs);
      current_state = HelpPeople;
      current_user.target = stairs;
    } else if (!strcmp(msg, "elevator")) {
      current_target = targetMap.find(elevator);
      current_state = HelpPeople;
      current_user.target = elevator;
    } else if (!strcmp(msg, "stop")) {
      current_target = targetMap.find(stop);
      current_state = HelpPeople;
      current_user.target = stop;
    } else if (!strcmp(msg, "guide")) {
      find_plan();
      current_state = GuidePeople;
      current_user.target = guide;
    }

    if (current_target == HelpPeople || current_state == GuidePeople) {
      // pub 
      std_msgs::String message;
      std::stringstream ss;
      ss << current_target.x << current_target.y << current_target.rx << current_target.ry;
      message.data = ss.str();
      go_to_target.publish(message);
    }
    
  }
}

/*
  HelpPeople callback
 */

void helping_people(const std_msgs::String::ConstPtr& msg) {
  if (current_state == GuidePeople) {
    if (current_user.face < THRESHOLD) {
      current_state = HelpingWhileWaitForPerson;
      current_wait_user = current_user;
      return;
    }

    if (reach_target) {
      tts("we reached " + current_user.target);
      current_state = Patrol;
      return;
    }

    // control the speed
    std_msgs::String message;
    std::stringstream ss;
    ss << 1./current_user.face;
    message.data = ss.str();
    go_to_target.publish(message);
  }
}

/*
  HelpingWhileWaitForPerson callback
 */

void wait_helping_people(const std_msgs::String::ConstPtr& msg) {
  if (current_user.face > THRESHOLD && current_user.id == current_wait_user.id) {
    current_state = HelpPeople;
    timeOutCount = 0;
  }
}

/*
  GuidePeople callback not implemented
 */


int main(int argc, char **argv) {
  ros::init(argc, argv, "state");

  srand( time(NULL) );
  go_to_target = n.advertise<std_msgs::String>("go_to_target", 1000);
  ros::Rate loop_rate(10);

  int count = 0;

  ros::Subscriber sub1 = n.subscribe("hear_current_pose", 1000, hear_current_pose);
  ros::Subscriber sub2 = n.subscribe("last_see_people", 1000, last_see_people);
  ros::Subscriber sub3 = n.subscribe("last_see_people", 1000, helping_people);
  ros::Subscriber sub4 = n.subscribe("last_see_people", 1000, wait_helping_people);
  ros::Subscriber sub5 = n.subscribe("finish_construction_Callback", 1000, finish_construction_Callback);
  ros::Subscriber sub6 = n.subscribe("hear_find_target_Callback", 1000, hear_find_target_Callback);
  ros::Subscriber sub7 = n.subscribe("listen_to_people", 1000, listen_to_people);
  
  while (ros::ok()) {
    ros::spinOnce();

    loop_rate.sleep();

    switch (current_state):
      Patrol:
        // random move around;
        if (timeOutCount < 50) {
          timeOutCount++;
          continue;
        }
        std_msgs::String msg;
        std::stringstream ss;
        ss << current_pose.x + (double) 10* rand() / (RAND_MAX + 1.0) << 
              current_pose.y + (double) 10* rand() / (RAND_MAX + 1.0) <<
              current_pose.rx + (double) 10* rand() / (RAND_MAX + 1.0) <<
              current_pose.ry + (double) 10* rand() / (RAND_MAX + 1.0);
        msg.data = ss.str();
        chatter_pub.publish(msg);
        timeOutCount = 0;
      WaitForReplyWhilePatrol:
        if (timeOutCount > 50) {
          timeOutCount = 0;
          current_state = Patrol;
          break;
        }
        if (timeOutCount % 10 == 0) {
          tts("Are you going to " + user.target + "?");
        }
        timeOutCount++;
      HelpingWhileWaitForPerson:
        if (timeOutCount > 50) {
          timeOutCount = 0;
          lost_user.insert(current_wait_user.id, current_wait_user);
          current_state = Patrol;
          break;
        }
        if (timeOutCount % 10 == 0) {
          tts("You still there ?");
        }
        timeOutCount++;
      GuidingWhileWaitForPerson:
        if (timeOutCount > 50) {
          timeOutCount = 0;
          lost_user.insert(current_wait_user.id, current_wait_user);
          current_state = Patrol;
          break;
        }
        if (timeOutCount % 10 == 0) {
          tts("You still there ?");
        }
        timeOutCount++;
      default:
        break;
  }

  return 0;
}