#include <ros/ros.h>
#include <string>
#include<geometry_msgs/Pose2D.h>
#include "muti_planning/agentgoal.h"
#include "common.h"

class Agent
{
	public:
		Agent();
	private:
		bool goal_callback(muti_planning::agentgoal::Request &req, 
						   				 muti_planning::agentgoal::Response &res);

		void timeCallback(const ros::TimerEvent& event);
		ros::NodeHandle nh_;
		ros::Publisher pub_;
		ros::ServiceServer goal_;
		geometry_msgs::Pose2D state;
		ros::Timer timer;
};

Agent::Agent()
{
	pub_ = nh_.advertise<geometry_msgs::Pose2D>("agent_feedback" , 1);
	goal_ = nh_.advertiseService("update_goal", &Agent::goal_callback,this);
	timer = nh_.createTimer(ros::Duration(0.1), &Agent::timeCallback, this);
}

void Agent::timeCallback(const ros::TimerEvent& event)
{
	// publish the agent state
	pub_.publish(state);
}

bool Agent::goal_callback(muti_planning::agentgoal::Request &req, 
									 				muti_planning::agentgoal::Response &res)
{
	
	state.x = req.start.x;
	state.y = req.start.x;
	state.theta = req.start.theta;
	return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "agent");
  Agent agent;
  ros::spin();
  return 0;
}
