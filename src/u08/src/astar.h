#ifndef ASTAR_H_
#define ASTAR_H_

#include <vector>
#include <queue>

enum State {OPEN, CLOSED, UNKNOWN};

class Node {
public:
	Node(int _x, int _y);
	int x, y, w, h;
	State state;
	Node *from;

};

class C {
public:
	bool operator()(Node *n1, Node *n2) {
		return n1->h > n2->h;
	}
};

class Map {
public:
	Map(int width, int height);
	~Map();
	int width, height;
	Node **nodes;
	std::priority_queue<Node*, std::vector<Node*>, C> open;
	void setObstacle(int x, int y);
	void astar(int x_s, int y_s, int x_t, int y_t);
	Node* &operator[](int y) {
		return nodes[y];
	}
};

#endif /* ASTAR_H_ */
