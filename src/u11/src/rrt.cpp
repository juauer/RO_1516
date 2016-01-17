#include <stdlib.h>
#include "rrt.h"

std::vector<RRT::Pose> RRT::rrt(Pose &pose_start, Pose &pose_goal, int its, float step,
		Pose (*sample)(), bool (*blocked)(Pose &pose)) {
	std::vector<Node*> nodes;
	Node *node_start = new Node(pose_start, NULL);
	nodes.push_back(node_start);

	for(int i=0; i<its; ++i) {
		Pose pose_rand  = (*sample)();
		Node *node_near = nodes.front();
		float dist  = pose_rand.distance(node_near->pose);

		for(std::vector<Node*>::const_iterator node=nodes.begin(); node!=nodes.end(); ++node) {
			float d = pose_rand.distance((*node)->pose);

			if(d < dist) {
				node_near = *node;
				dist   = d;
			}
		}

		Node *node_new = new Node(Pose(pose_rand.x, pose_rand.y), node_near);

		if(dist > step) {
			node_new->pose.x = node_near->pose.x + (pose_rand.x-node_near->pose.x)*step/dist;
			node_new->pose.y = node_near->pose.y + (pose_rand.y-node_near->pose.y)*step/dist;
		}

		if(!(*blocked)(node_new->pose))
			nodes.push_back(node_new);
	}

	Node *node_near = nodes.front();
	float dist      = pose_goal.distance(node_near->pose);

	for(std::vector<Node*>::const_iterator node=nodes.begin(); node!=nodes.end(); ++node) {
		float d = pose_goal.distance((*node)->pose);

		if(d < dist) {
			node_near = *node;
			dist      = d;
		}
	}

	std::vector<Pose> path;
	path.push_back(pose_goal);

	while(node_near->parent != NULL) {
		path.push_back(node_near->pose);
		node_near = node_near->parent;
	}

	return path;
}
