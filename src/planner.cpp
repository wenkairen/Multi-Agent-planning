#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include "muti_planning/agentpath.h"
#include "roadmap.h"
#include <string>

//fisrt agent path wiil be second agent obstical;
vector<vector<int>> obstical_map(11, vector<int>(11, 0));	

class Planner
{
	public:
		Planner();
	private:
		void timeCallback(const ros::TimerEvent& event);
		void state_Callback(const geometry_msgs::Pose2D::ConstPtr& msg);
		bool path_callback(muti_planning::agentpath::Request &req, muti_planning::agentpath::Response &res);
		vector<Node::Ptr > reconstructPath(Node::Ptr start, Node::Ptr end, vector<vector<Node::Ptr>> came_from);
		void vis_marker();
		vector<vector<Node::Ptr>> findPath(Node::Ptr start, Node::Ptr end, vector<Node::Ptr> &map);
		double heuristic(Node::Ptr state, Node::Ptr end);
		ros::NodeHandle nh_;
		ros::Publisher pub_;
		ros::Publisher vis_pos_;
		ros::Publisher vis_path_;
		ros::Subscriber sub_;
		ros::ServiceServer plan_service;
		// map info : start and goal
		RoadMap::map_Ptr rm = make_shared<RoadMap>();
		vector<int> map_size;
		
		//path
		geometry_msgs::Pose pose;
		nav_msgs::Path final_path;
		Node::Ptr start = make_shared<Node>(0);
		Node::Ptr end = make_shared<Node>(0);

		//visualize
		visualization_msgs::MarkerArray smkra;
		visualization_msgs::MarkerArray gmkra;
		visualization_msgs::MarkerArray path_marker;
		ros::Timer timer;
		std::string agent_id;
		
};

Planner::Planner()
{
	//acutal path for each agent
	pub_ = nh_.advertise<nav_msgs::Path>("get_map" , 1);
	// visual maker for each agent
	vis_pos_ = nh_.advertise<visualization_msgs::MarkerArray>( "vis_st_g", 2 );
	vis_path_ = nh_.advertise<visualization_msgs::MarkerArray>( "vis_path", 1 );
	// sub to teh agent current position
	sub_ = nh_.subscribe<geometry_msgs::Pose2D>("agent_feedback", 10, &Planner::state_Callback, this);
	// wait for the goal
	plan_service = nh_.advertiseService("get_plan", &Planner::path_callback, this);
	timer = nh_.createTimer(ros::Duration(0.1), &Planner::timeCallback, this);
	// map info
	rm->buildMap();
	map_size = rm->getSize();


}

void Planner::timeCallback(const ros::TimerEvent& event)
{
	pub_.publish(final_path);
	
	// for visual
	vis_pos_.publish(smkra);
	vis_pos_.publish(gmkra);
	vis_path_.publish(path_marker);

}
void Planner::vis_marker(){
	
		
	// Visualize Start Point
	visualization_msgs::Marker smkr;
	smkr.id = 0;
	smkr.ns = agent_id;
	smkr.type = visualization_msgs::Marker::CYLINDER;
	smkr.action = visualization_msgs::Marker::ADD;
	smkr.header.stamp = ros::Time();
	smkr.header.frame_id = "/map";	
	smkr.scale.x = 0.5;
	smkr.scale.y = 0.5;
	smkr.scale.z = 1.0;
	smkr.pose.position.x = start->x;
	smkr.pose.position.y = start->y;
	smkr.pose.position.z = 0;
	smkr.pose.orientation.x = 0.0;
	smkr.pose.orientation.y = 0.0;
	smkr.pose.orientation.z = 1.0;
	smkr.pose.orientation.w = 0.0;	
	smkr.color.a = 0.5; 
	smkr.color.r = 1.0;
	smkr.color.g = 0.0;
	smkr.color.b = 0.0;
	smkra.markers.push_back(smkr);

	//Visualize Goal Point
	visualization_msgs::Marker gmkr;
	gmkr.id = 1;
	//smkr.type = visualization_msgs::Marker::MESH_RESOURCE;
	gmkr.type = visualization_msgs::Marker::CUBE;
	gmkr.ns = agent_id;
	gmkr.action = visualization_msgs::Marker::ADD;
	gmkr.header.stamp = ros::Time();
	gmkr.header.frame_id = "/map";	
	gmkr.scale.x = 0.5;
	gmkr.scale.y = 0.5;
	gmkr.scale.z = 1.0;
	gmkr.pose.position.x = end->x;
	gmkr.pose.position.y = end->y;
	gmkr.pose.position.z = 0;
	gmkr.pose.orientation.x = 0.0;
	gmkr.pose.orientation.y = 0.0;
	gmkr.pose.orientation.z = 1.0;
	gmkr.pose.orientation.w = 0.0;
	gmkr.color.a = 0.5; 
	gmkr.color.r = 0.0;
	gmkr.color.g = 0.0;
	gmkr.color.b = 1.0;
	gmkra.markers.push_back(gmkr);

	visualization_msgs::Marker path_marker_ai;
	path_marker_ai.ns = agent_id;
	path_marker_ai.header.frame_id = "/map";
	path_marker_ai.type = visualization_msgs::Marker::LINE_LIST;
	path_marker_ai.action = visualization_msgs::Marker::ADD;
	path_marker_ai.scale.x = 0.1;
	path_marker_ai.scale.y = 0.1;
	path_marker_ai.scale.z = 0.1;
	path_marker_ai.color.a = 1.0;
	path_marker_ai.color.r = 1.0;
	path_marker_ai.color.g = 1.0;
	path_marker_ai.color.b = 0.0;
	path_marker_ai.pose.orientation.w = 1.0;
	path_marker_ai.pose.position.x = 0;
	path_marker_ai.pose.position.y = 0;
	path_marker_ai.pose.position.z = 0;

	for(int i = 0; i < final_path.poses.size(); i++){
		geometry_msgs::Point temp_point;
		temp_point.x = final_path.poses[i].pose.position.x;
		temp_point.y = final_path.poses[i].pose.position.y;
		temp_point.z = 0;
		path_marker_ai.points.push_back(temp_point);
	}
	if (final_path.poses.size()%2){
		geometry_msgs::Point temp_point;
		temp_point.x = final_path.poses[final_path.poses.size()-1].pose.position.x;
		temp_point.y = final_path.poses[final_path.poses.size()-1].pose.position.y;
		temp_point.z = 0;
		path_marker_ai.points.push_back(temp_point);
	}
	path_marker.markers.push_back(path_marker_ai);
}


void Planner::state_Callback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	start->x = msg->x;
	start->y = msg->y;
	start->yaw = msg->theta;
	// cout << start->x << endl;

}

double Planner::heuristic(Node::Ptr state, Node::Ptr end)
{
	int xs = state->x, ys = state->y;
	int xg = end->x, yg = end->y;
	double dist = (xg - xs)*(xg - xs) + (yg - ys)*(yg - ys);

	return dist;
}


bool Planner::path_callback(muti_planning::agentpath::Request &req, 
									 muti_planning::agentpath::Response &res)
{  


	agent_id = req.id;
	geometry_msgs::Pose2D goal = req.goal;

	
	end->x = goal.x;
	end->y = goal.y;
	end->yaw = goal.theta;
	cout << end->x << endl;
	vector<vector<Node::Ptr>> came_from = findPath(start, end, rm->map_);
	vector<Node::Ptr > node_path = reconstructPath(start, end, came_from);

	// convert path to path message
	final_path.poses = vector<geometry_msgs::PoseStamped>(node_path.size());
	std_msgs::Header header_msg;
	header_msg.stamp = ros::Time::now();
	header_msg.frame_id = "map";
	final_path.header = header_msg;

	for(int i = 0; i < node_path.size(); i++){
		pose.position.x = node_path[i]->x;
		pose.position.y = node_path[i]->y;
		final_path.poses[i].pose = pose;
	}
	// the start and goal of second agent is still accessible for
	// second agent, need reset here.
	obstical_map[start->x][start->y] = 0;
	obstical_map[start->x][start->y] = 0;

	// call for visual the agent path
	vis_marker();

  return true;
}

vector<Node::Ptr> Planner::reconstructPath(Node::Ptr start, Node::Ptr end, vector<vector<Node::Ptr>> came_from)
{
	vector<Node::Ptr> path;
	path.push_back(end);

	Node::Ptr current = came_from[end->x][end->y]; 
	int x = current->x;
	int y = current->y;
	while(x != start->x || y != start->y){
		path.push_back(current);
		current = came_from[x][y];
		x = current->x;
		y = current->y;
	}
	return path;
}

vector<vector<Node::Ptr>> Planner::findPath(Node::Ptr start, Node::Ptr end, vector<Node::Ptr> &map)
{
	// Here use basic A* search , also can be used for higher dimention.
	// define the path
	auto comp = [](const Node::Ptr n1, const Node::Ptr n2)
	{
		return (n1->cost) < (n2->cost);
	};


	vector<vector<int>> closed(map_size[0], vector<int>(map_size[1], 0));
	vector<vector<Node::Ptr>> came_from(map_size[0], vector<Node::Ptr>(map_size[1]));

	start->val = 0;
	start->cost = start->val + heuristic(start, end);

	closed[start->x][start->y] = 1;
	came_from[start->x][start->y] = start;

	vector<Node::Ptr> q = {start}; // open list 
	bool finished = false;

	while(!q.empty()){
    std::sort(q.begin(), q.end(), comp);
    //grab first elment
    Node::Ptr curr = q[0];
    //pop first element
		q.erase(q.begin()); 

		// the fist agent path will be second agent's obstical
		obstical_map[curr->x][curr->y] = 100;
		//goal found
		if(curr->x == end->x && curr->y == end->y){

			return came_from;
		}

		for(int i = 0; i < map.size(); i++){
			if(curr->x == map[i]->x && curr->y == map[i]->y){
				// find the neightboor of inside map;
				Node::Ptr nb = map[i];
				double g = nb->val;
				while(nb->next != NULL){
					nb = nb->next;
					int x2 = nb->x;
					int y2 = nb->y;
					double g2 = g + 10; // step cost

					if(obstical_map[nb->x][nb->y] != 100 && closed[nb->x][nb->y] == 0){
						nb->val = g2; // current g
						nb->cost = g2 + heuristic(nb, end);
						closed[x2][y2] = 1;
						q.push_back(nb);
						came_from[x2][y2] = map[i];
					}
				}
			}
		}
	}

	cout << "no valid path." << endl;
	return came_from;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "planner");
  Planner planner;
	ros::spin();
  return 0;
}
