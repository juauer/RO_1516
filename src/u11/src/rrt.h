#ifndef RRT_H_
#define RRT_H_

#include <vector>
#include <math.h>

namespace RRT {
class Pose {
public:
	float x, y;

	Pose(float _x, float _y) :
		x(_x), y(_y) {};

	float distance(const Pose &p) {
		return sqrtf((x-p.x)*(x-p.x) + (y-p.y)*(y-p.y));
	};
};

class Node {
public:
	Pose pose;
	Node *parent;

	Node(Pose _pose, Node *_parent) :
		pose(_pose), parent(_parent) {};
};

std::vector<Pose> rrt(Pose &pose_start, Pose &pose_goal, int its, float step,
	Pose (*sample)(), bool (*blocked)(Pose &pose));
}

#endif /* RRT_H_ */
