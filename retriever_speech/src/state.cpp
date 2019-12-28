#include "ros/ros.h"
#include "std_msgs/String.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "nav_msgs/Odometry.h"
#include "actionlib_msgs/GoalID.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "retriever_speech/user_info.h"
#include <time.h> 
#include <chrono>
#include <sstream>
#include <map>
#include <vector>
#include <string>
#include <math.h>
#define THRESHOLD 10000

using namespace std;

enum State {
  MapConstruction, 
  Patrol,
  WaitForReplyWhilePatrol,
  HelpingWhileWaitForPerson,
  HelpPeople,
  GuidingWhileWaitForPerson,
  GuidePeople,
};

enum Place {
  bathroom,
  water_dispenser,
  stairs,
  elevator,
  stop,
  guide
};

typedef struct {
  double x;
  double y;
  double rx;
  double ry;
} P;

typedef struct {
  int id;
  int face;
  Place target;
} U;

// public variable;
int current_state = MapConstruction;
int timeOutCount = 0;
bool arrived = false;
P current_pose;
P current_target;
U current_wait_user;
U current_user;
map<int, U> lost_user; 
map<Place, P> targetMap;
//vector<P> route;
ros::NodeHandle n;
ros::Publisher go_to_target;
ros::Publisher cancel;

void tts(string s) {
  ;
}

void find_plan() {
  // find route based on current position
  ;
}

void reach_target(const actionlib_msgs::GoalStatusArray::ConstPtr& msg) {
  if (msg->status_list[0].status == 3 && (msg->status_list[0].text!="Goal reached.")) {
    arrived = true;
  }
}

/*
  Report constantly
 */

// keep track of current position
void hear_current_pose(const nav_msgs::Odometry::ConstPtr& msg) {
  if (current_state == HelpPeople || current_state == GuidePeople 
    || current_state == WaitForReplyWhilePatrol) return;
  current_pose.x = msg->pose.pose.position.x;
  current_pose.y = msg->pose.pose.position.y;
  current_pose.rx = msg->pose.pose.orientation.z;
  current_pose.ry = msg->pose.pose.orientation.w;
}

// keep track of person saw
void last_see_people(const retriever_speech::user_info::ConstPtr& msg) {
  current_user.id = msg->user_id;
  current_user.face = msg->face_area;
}

/*
  Map Construction callback
 */

// find some targets
void hear_find_target_Callback(const std_msgs::String::ConstPtr& msg) {
  if (current_state != MapConstruction) return;
  if (!strcmp(msg->data.c_str(), "bathroom")) {
    targetMap[bathroom] = current_pose;
  } else if (!strcmp(msg->data.c_str(), "water_dispenser")) {
    targetMap[water_dispenser] = current_pose;
  } else if (!strcmp(msg->data.c_str(), "stairs")) {
    targetMap[stairs] = current_pose;
  } else if (!strcmp(msg->data.c_str(), "elevator")) {
    targetMap[elevator] = current_pose;
  } else if (!strcmp(msg->data.c_str(), "stop")) {
    targetMap[stop] = current_pose;
  }
}

/*
  Patrol Construction callback
 */

void listen_to_people(const std_msgs::String::ConstPtr& msg) {
  if (current_state == WaitForReplyWhilePatrol) {
    if (!strcmp(msg->data.c_str(), "yes")) {
      if (current_wait_user.target == guide) {
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
  if (current_state == Patrol && (current_user.face > THRESHOLD)) {
    // check if person is lost
    U user;
    auto tmp = lost_user.find(current_user.id);
    if (tmp != lost_user.end()) {
      user = tmp->second;
      current_state = WaitForReplyWhilePatrol;
      current_wait_user = user;
      return;
    }

    if (!strcmp(msg->data.c_str(), "bathroom")) {
      current_state = HelpPeople;
      current_user.target = bathroom;
      auto tmp = targetMap.find(current_user.target);
      if (tmp != targetMap.end()) {
        current_target = tmp->second;
      }
    } else if (!strcmp(msg->data.c_str(), "water_dispenser")) {
      current_state = HelpPeople;
      current_user.target = water_dispenser;
      auto tmp = targetMap.find(current_user.target);
      if (tmp != targetMap.end()) {
        current_target = tmp->second;
      }
    } else if (!strcmp(msg->data.c_str(), "stairs")) {
      current_state = HelpPeople;
      current_user.target = stairs;
      auto tmp = targetMap.find(current_user.target);
      if (tmp != targetMap.end()) {
        current_target = tmp->second;
      }
    } else if (!strcmp(msg->data.c_str(), "elevator")) {
      current_state = HelpPeople;
      current_user.target = elevator;
      auto tmp = targetMap.find(current_user.target);
      if (tmp != targetMap.end()) {
        current_target = tmp->second;
      }
    } else if (!strcmp(msg->data.c_str(), "stop")) {
      current_state = HelpPeople;
      current_user.target = stop;
      auto tmp = targetMap.find(current_user.target);
      if (tmp != targetMap.end()) {
        current_target = tmp->second;
      }
    } else if (!strcmp(msg->data.c_str(), "guide")) {
      // find_plan();
      current_state = GuidePeople;
      current_user.target = guide;
    }

    if (current_state == HelpPeople || current_state == GuidePeople) {
      // pub 
      arrived = false;
      move_base_msgs::MoveBaseActionGoal s;
      s.goal.target_pose.pose.position.x = current_target.x;
      s.goal.target_pose.pose.position.y = current_target.y;
      s.goal.target_pose.pose.orientation.z = current_target.rx;
      s.goal.target_pose.pose.orientation.w = current_target.ry;
      go_to_target.publish(s);
    }
    
  }
}

/*
  HelpPeople callback
 */

void helping_people(const retriever_speech::user_info::ConstPtr& msg) {
  if (current_state == GuidePeople) {
    if (msg->face_area < THRESHOLD) {
      current_state = HelpingWhileWaitForPerson;
      current_wait_user = current_user;
      actionlib_msgs::GoalID temp;
      cancel.publish(temp);    
      return;
    }

    if (arrived) {
      tts("we reached ");
      current_state = Patrol;
      actionlib_msgs::GoalID temp;
      cancel.publish(temp);    
      return;
    }
  }
}

/*
  HelpingWhileWaitForPerson callback
 */

void wait_helping_people(const retriever_speech::user_info::ConstPtr& msg) {
  if (msg->face_area > THRESHOLD && msg->user_id == current_wait_user.id) {
    current_state = HelpPeople;
    timeOutCount = 0;
    auto tmp = targetMap.find(current_user.target);
    if (tmp != targetMap.end()) {
      current_target = tmp->second;
    }

    move_base_msgs::MoveBaseActionGoal message;
    message.goal.target_pose.pose.position.x = current_target.x;
    message.goal.target_pose.pose.position.y = current_target.y;
    message.goal.target_pose.pose.orientation.z = current_target.rx;
    message.goal.target_pose.pose.orientation.w = current_target.ry;
    go_to_target.publish(message);
  }
}

/*
  GuidePeople callback not implemented
 */


int main(int argc, char **argv) {
  ros::init(argc, argv, "state");

  srand( time(NULL) );

  bool finished;
  
  go_to_target = n.advertise<std_msgs::String>("/move_base/goal", 1000);
  cancel = n.advertise<std_msgs::String>("/move_base/cancel", 1000);
  
  ros::Rate loop_rate(10);

  int count = 0;

  ros::Subscriber sub1 = n.subscribe("/odom", 1000, hear_current_pose);
  ros::Subscriber sub2 = n.subscribe("/last_see_people", 1000, last_see_people);
  ros::Subscriber sub3 = n.subscribe("/last_see_people", 1000, helping_people);
  ros::Subscriber sub4 = n.subscribe("/last_see_people", 1000, wait_helping_people);
  ros::Subscriber sub6 = n.subscribe("/hear_find_target_Callback", 1000, hear_find_target_Callback);
  ros::Subscriber sub7 = n.subscribe("/listen_to_people", 1000, listen_to_people);
  ros::Subscriber sub8 = n.subscribe("/move_base/result", 1000, reach_target);
  
  while (ros::ok()) {
    ros::spinOnce();

    loop_rate.sleep();

    ros::param::get("finish_construction", finished);
    if (finished) {
      current_state = Patrol;
    }

    switch (current_state) {
      Patrol:
        // random move around;
        if (timeOutCount > 50) {
          actionlib_msgs::GoalID temp;
          cancel.publish(temp);
          timeOutCount = 0;
        } 
        if (timeOutCount == 0) {
          move_base_msgs::MoveBaseActionGoal message;
          message.goal.target_pose.pose.position.x = current_pose.x + (double) 10* rand() / (RAND_MAX + 1.0);
          message.goal.target_pose.pose.position.y = current_pose.y + (double) 10* rand() / (RAND_MAX + 1.0);
          message.goal.target_pose.pose.orientation.z = current_pose.rx + (double) 10* rand() / (RAND_MAX + 1.0);
          message.goal.target_pose.pose.orientation.w = current_pose.ry + (double) 10* rand() / (RAND_MAX + 1.0);
          go_to_target.publish(message);
        }
        timeOutCount++;
      WaitForReplyWhilePatrol:
        if (timeOutCount > 50) {
          timeOutCount = 0;
          current_state = Patrol;
          break;
        }
        if (timeOutCount % 20 == 0) {
          tts("You are the guy lost before, do you want to keep going?");
        }
        timeOutCount++;
      HelpingWhileWaitForPerson:
        if (timeOutCount > 50) {
          timeOutCount = 0;
          lost_user[current_wait_user.id] = current_wait_user;
          current_state = Patrol;
          break;
        }
        if (timeOutCount % 10 == 0) {
          tts("Hello? stay behind me!!!");
        }
        timeOutCount++;
      GuidingWhileWaitForPerson:
        if (timeOutCount > 50) {
          timeOutCount = 0;
          lost_user[current_wait_user.id] = current_wait_user;
          current_state = Patrol;
          break;
        }
        if (timeOutCount % 10 == 0) {
          tts("Hello? stay behind me!!!");
        }
        timeOutCount++;
      default:
        break;
    }
  }

  return 0;
}
