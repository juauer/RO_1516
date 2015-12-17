#ifndef ASTAR_H_
#define ASTAR_H_

#include <vector>
#include <queue>

class Pose {
public:
	float x, y, ox, oy;

	Pose(float _x, float _y, float _ox, float _oy) :
		x(_x), y(_y), ox(_ox), oy(_oy) {};
};

class Node {
public:
	Pose pose;
	Node *parent;
	double costs_g, costs_f;

	Node(Pose &_pose, Node *_parent,
			double (*g)(Pose &start, Pose &goal),
			double (*h)(Pose &pose));
	Node(Pose &_pose, double (*h)(Pose &pose));
};

class C {
public:
	bool operator()(Node *n1, Node *n2) {
		return n1->costs_f > n2->costs_f;
	}
};

class AStar {
public:
	std::priority_queue<Node*, std::vector<Node*>, C> open;

	AStar(Pose &pose_start, Pose &pose_goal,
			std::vector<Pose> (*neighbors)(Pose &parent),
			double (*g)(Pose &start, Pose &goal),
			double (*h)(Pose &pose),
			bool (*goalReached)(Pose &pose));
};

#endif /* ASTAR_H_ */
