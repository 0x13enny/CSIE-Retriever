#include "ros/ros.h"
#include "std_msgs/String.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "move_base_msgs/MoveBaseActionResult.h"
#include "nav_msgs/Odometry.h"
#include "actionlib_msgs/GoalID.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "retriever_speech/user_info.h"
#include <time.h> 
#include <stdlib.h>
#include <cstdlib> 
#include <chrono>
#include <sstream>
#include <map>
#include <vector>
#include <string>
#include <math.h>
#define THRESHOLD 30000

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
ros::Publisher go_to_target;
ros::Publisher cancel;

void tts(const string& s) {
  string cmd = "play -v 10 /home/benny/Documents/retriever_ws/src/CSIE-Retriever/voice/" + s + ".mp3";
  system(cmd.c_str());
}

void find_plan() {
  // find route based on current position
  ;
}

void reach_target(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg) {
  if (current_state == HelpPeople && msg->status.status == 3 && (msg->status.text == "Goal reached.")) {
    if (current_user.target == bathroom) {
      tts("bathroom");
    } else if (current_user.target == stairs) {
      tts("stairs");
    } else if (current_user.target == elevator) {
      tts("elevator");
    } 
    ROS_INFO("reach the target: HelpPeople --> Patrol");
    current_state = Patrol;
    actionlib_msgs::GoalID temp;
    cancel.publish(temp);    
    return;
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
//  ROS_INFO("%lf,%lf",current_pose.x, current_pose.y);
}

// keep track of person saw
void last_see_people(const retriever_speech::user_info::ConstPtr& msg) {
  current_user.id = 0;
  current_user.face = (int)msg->face_area;
  ROS_INFO("detect user: %d, area: %d", current_user.id, current_user.face);
}

/*
  Map Construction callback
 */

// find some targets
void hear_find_target_Callback(const std_msgs::String::ConstPtr& msg) {
  printf("%s",msg->data.c_str());
  printf("%d",current_state == MapConstruction);
  if (current_state != MapConstruction) return;
  if (!strcmp(msg->data.c_str(), "bathroom")) {
    ROS_INFO("detect bathroom at %lf, %lf", current_pose.x, current_pose.y);
    targetMap[bathroom] = current_pose;
  } else if (!strcmp(msg->data.c_str(), "stairs")) {
    ROS_INFO("detect stairs at %lf, %lf", current_pose.x, current_pose.y);
    targetMap[stairs] = current_pose;
  } else if (!strcmp(msg->data.c_str(), "elevator")) {
    ROS_INFO("detect elevator at %lf, %lf", current_pose.x, current_pose.y);
    targetMap[elevator] = current_pose;
  }
}

void has_face(const retriever_speech::user_info::ConstPtr& msg) {
  if (current_state == Patrol && (msg->face_area > THRESHOLD)) {
  //  ROS_INFO("lost hear voice: face > THRESHOLD");
    // check if person is lost
    U user;
    auto tmp = lost_user.find(current_user.id);
    if (tmp != lost_user.end()) {
      user = tmp->second;
      current_state = WaitForReplyWhilePatrol;
      timeOutCount = 0;
      current_wait_user = user;
      ROS_INFO("wait for reply");
      return;
    }
  }
}
/*
  Patrol Construction callback
 */

void listen_to_people(const std_msgs::String::ConstPtr& msg) {
  if (current_state == WaitForReplyWhilePatrol) {
    if (!strcmp(msg->data.c_str(), "yesss")) {
      ROS_INFO("lost person reply yes");
      current_state = HelpPeople;
      lost_user.erase(current_wait_user.id);
      timeOutCount = 0;
    } else if (!strcmp(msg->data.c_str(), "noooo")) {
      ROS_INFO("lost person reply no");
      lost_user.erase(current_wait_user.id);
      current_state = Patrol;
      timeOutCount = 0;
    }
    return;
  }
  if (current_state == Patrol && (current_user.face > THRESHOLD)) {
    ROS_INFO("hear voice: face > THRESHOLD");

    if (!strcmp(msg->data.c_str(), "bathroom")) {
      current_state = HelpPeople;
      current_user.target = bathroom;
      auto tmp = targetMap.find(current_user.target);
      if (tmp != targetMap.end()) {
        current_target = tmp->second;
      }
      ROS_INFO("hear go to bathroom");
    } else if (!strcmp(msg->data.c_str(), "stairs")) {
      current_state = HelpPeople;
      current_user.target = stairs;
      auto tmp = targetMap.find(current_user.target);
      if (tmp != targetMap.end()) {
        current_target = tmp->second;
      }
      ROS_INFO("hear go to stairs");
    } else if (!strcmp(msg->data.c_str(), "elevator")) {
      current_state = HelpPeople;
      current_user.target = elevator;
      auto tmp = targetMap.find(current_user.target);
      if (tmp != targetMap.end()) {
        current_target = tmp->second;
      }
      ROS_INFO("hear go to elevator");
    } else if (!strcmp(msg->data.c_str(), "guide")) {
      // find_plan();
      current_state = GuidePeople;
      current_user.target = guide;
      ROS_INFO("hear guide");
    }

    if (current_state == HelpPeople || current_state == GuidePeople) {
      // pub 
      tts("stay_close");
      move_base_msgs::MoveBaseActionGoal s;
      s.goal.target_pose.pose.position.x = current_target.x;
      s.goal.target_pose.pose.position.y = current_target.y;
      s.goal.target_pose.pose.orientation.z = current_target.rx;
      s.goal.target_pose.pose.orientation.w = current_target.ry;
      s.goal.target_pose.header.frame_id = "map";
      go_to_target.publish(s);
    }
    
  }
}

/*
  HelpPeople callback
 */

void helping_people(const retriever_speech::user_info::ConstPtr& msg) {
  if (current_state == HelpPeople) {
    if (msg->face_area < THRESHOLD) {
      ROS_INFO("face_area < THRESHOLD, Helping people --> Wait for people");
      current_state = HelpingWhileWaitForPerson;
      current_wait_user = current_user;
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
  if (msg->face_area > THRESHOLD && msg->user_id == current_wait_user.id && current_state == HelpingWhileWaitForPerson) {
    current_state = HelpPeople;
    timeOutCount = 0;
    auto tmp = targetMap.find(current_wait_user.target);
    if (tmp != targetMap.end()) {
      current_target = tmp->second;
    }

    ROS_INFO("face_area > THRESHOLD, HelpingWhileWaitForPerson --> HelpPeople");
    move_base_msgs::MoveBaseActionGoal message;
    message.goal.target_pose.pose.position.x = current_target.x;
    message.goal.target_pose.pose.position.y = current_target.y;
    message.goal.target_pose.pose.orientation.z = current_target.rx;
    message.goal.target_pose.pose.orientation.w = current_target.ry;
    message.goal.target_pose.header.frame_id = "map";
    go_to_target.publish(message);
  }
}

/*
  GuidePeople callback not implemented
 */


int main(int argc, char **argv) {
  ros::init(argc, argv, "state");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  srand( time(NULL) );

  bool finished;
  current_state = MapConstruction;
  
  go_to_target = n.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 1000);
  cancel = n.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1000);
  
  ros::Rate loop_rate(10);

  int count = 0;

  ros::Subscriber sub1 = n.subscribe("/aria_controller/pose", 1000, hear_current_pose);
  ros::Subscriber sub2 = n.subscribe("/last_see_people", 1000, last_see_people);
  ros::Subscriber sub0 = n.subscribe("/last_see_people", 1000, has_face);
  ros::Subscriber sub3 = n.subscribe("/last_see_people", 1000, helping_people);
  ros::Subscriber sub4 = n.subscribe("/last_see_people", 1000, wait_helping_people);
  ros::Subscriber sub6 = n.subscribe("/hear_find_target_Callback", 1000, hear_find_target_Callback);
  ros::Subscriber sub7 = n.subscribe("/listen_to_people", 1000, listen_to_people);
  ros::Subscriber sub8 = n.subscribe("/move_base/result", 1000, reach_target);

  while (ros::ok()) {
//    ROS_INFO("here");
    loop_rate.sleep();

    ros::param::get("finish_construction", finished);
    if (finished && count == 0) {
      current_state = Patrol;
      ROS_INFO("Construction --> Patrol");
      count = 1;
    }

    switch (current_state) {
      case Patrol:
        // random move around;
        if (timeOutCount > 150) {
          move_base_msgs::MoveBaseActionGoal message;
          message.goal.target_pose.pose.position.x = current_pose.x + (double) 0.5* rand() / (RAND_MAX + 1.0);
          message.goal.target_pose.pose.position.y = current_pose.y + (double) 0.5* rand() / (RAND_MAX + 1.0);
          message.goal.target_pose.pose.orientation.z = current_pose.rx + (double) 0.1* rand() / (RAND_MAX + 1.0);
          message.goal.target_pose.pose.orientation.w = current_pose.ry + (double) 0.1* rand() / (RAND_MAX + 1.0);
          message.goal.target_pose.header.frame_id = "map";
          go_to_target.publish(message);
        } 
        if (timeOutCount == 250) {
	  actionlib_msgs::GoalID temp;
          cancel.publish(temp);
          timeOutCount = 0;
        }
        timeOutCount++;
        break;
      case WaitForReplyWhilePatrol:
        if (timeOutCount > 600) {
          ROS_INFO("Timeout, Not replying, WaitForReplyWhilePatrol -> Patrol");
          timeOutCount = 0;
          current_state = Patrol;
          break;
        }
        if (timeOutCount % 100 == 0) {
          if (current_wait_user.target == bathroom) {
            tts("resume_bathroom");
          } else if (current_wait_user.target == stairs) {
            tts("resume_stairs");
          } else if (current_wait_user.target == elevator) {
            tts("resume_elevator");
          } 
        }
        timeOutCount++;
        break;
      case HelpingWhileWaitForPerson:
        if (timeOutCount > 300) {
          ROS_INFO("Timeout, Person lost, HelpingWhileWaitForPerson --> Patrol");
          ROS_INFO("add lost user %d %d", current_wait_user.id, current_wait_user.target);
          timeOutCount = 0;
          lost_user[current_wait_user.id] = current_wait_user;
          current_state = Patrol;
          break;
        }
        if (timeOutCount % 100 == 0) {
          tts("stay_close");
        }
        timeOutCount++;
        break;
      case GuidingWhileWaitForPerson:
        if (timeOutCount > 300) {
	        ROS_INFO("add lost user %d %d",current_wait_user.id, current_wait_user.target);
          timeOutCount = 0;
          lost_user[current_wait_user.id] = current_wait_user;
          current_state = Patrol;
          break;
        }
        if (timeOutCount % 100 == 0) {
          tts("stay_close");
        }
        timeOutCount++;
        break;
      default:
        break;
    }
//    ros::spin();
  }

  return 0;
}
