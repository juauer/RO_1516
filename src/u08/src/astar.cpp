#include "astar.h"

#include <stdio.h>
#include <string.h>
#include "opencv2/opencv.hpp"

const std::string path_captures = "./captures/";

Node::Node(int _x, int _y) :
			x(_x),
			y(_y),
			w(INT32_MAX),
			h(INT32_MAX),
			state(UNKNOWN),
			from(NULL)
{

}

Map::Map(int _width, int _height) :
			width(_width),
			height(_height)
{
	nodes = new Node*[height];

	for(int i=0; i<height; ++i) {
		nodes[i] = (Node*)calloc(width, sizeof(Node));

		for(int j=0; j<width; ++j)
			nodes[i][j] = Node(j, i);
	}
}

Map::~Map() {

}

void Map::setObstacle(int x, int y) {
	nodes[y][x].w = -1;
	nodes[y][x].h = -1;
	nodes[y][x].state  = CLOSED;
}

void Map::astar(int x_s, int y_s, int x_t, int y_t) {
	nodes[y_s][x_s].w = 0;
	nodes[y_s][x_s].h = abs(x_t - x_s) + abs(y_t - y_s);
	open.push(&nodes[y_s][x_s]);

	while(!open.empty()) {
		Node *n = open.top();
		open.pop();

		if(n->state == CLOSED)
			continue;

		n->state = CLOSED;

		if(n == &nodes[y_t][x_t])
			return;

		for(int y=std::max(0, n->y-1); y<std::min(height, n->y+2); ++y)
			for(int x=std::max(0, n->x-1); x<std::min(width, n->x+2); ++x) {
				if(nodes[y][x].state == CLOSED || (y!=n->y && x!=n->x))
					continue;

				int costs = n->w + 1;

				if(costs < nodes[y][x].w) {
					nodes[y][x].w     = costs;
					nodes[y][x].h     = costs + abs(x_t - x) + abs(y_t - y);
					nodes[y][x].state = OPEN;
					nodes[y][x].from  = n;
					open.push(&nodes[y][x]);
				}
			}
	}
}

int main(int, char**) {
	int width  = 15;
	int height = 19;
	int x_s    = 5;
	int y_s    = 1;
	int x_t    = 5;
	int y_t    = 10;
	int s      = 19;
	int r      = s / 2;
	int o      = r + 1;
	cv::Scalar  white(255, 255, 255);
	cv::Scalar   grey(200, 200, 200);
	cv::Scalar   blue(200,   0,   0);
	cv::Scalar  green(  0, 200,   0);
	cv::Scalar    red(  0,   0, 200);
	cv::Scalar yellow(  0, 255, 255);

	for(int i=-1; i<2; ++i) {
		Map map(width, height);

		for(int j=3+i; j<8+i; ++j)
			map.setObstacle(j, 5);

		map.astar(x_s, y_s, x_t, y_t);
		cv::Mat img(s*width, s*height, CV_8UC3, white);

		for(int y=0; y<height; ++y) {
			for(int x=0; x<width; ++x) {
				if(map[y][x].w == -1)
					cv::circle(img, cv::Point(s*x+o, s*y+o), r, blue, -1);
				else if(map[y][x].state == OPEN)
					cv::circle(img, cv::Point(s*x+o, s*y+o), r, grey, -1);
				else if(map[y][x].state == CLOSED)
					cv::circle(img, cv::Point(s*x+o, s*y+o), r, red,  -1);
			}
		}

		cv::circle(img, cv::Point(s*x_s+o, s*y_s+o), r, green, -1);
		cv::circle(img, cv::Point(s*x_t+o, s*y_t+o), r, green, -1);
		Node *n = &map[y_t][x_t];

		while(n != &map[y_s][x_s]) {
			cv::line(img, cv::Point(s*n->x+o, s*n->y+o), cv::Point(s*n->from->x+o, s*n->from->y+o), yellow, 2);
			n = n->from;
		}

		std::stringstream title, path;
		title << "i=" << i;
		cv::namedWindow(title.str());
		cv::imshow(title.str(), img);
		path << path_captures << "capture_2-" << (i+2) << ".png";
		cv::imwrite(path.str(), img);
	}

	cv::waitKey(0);
	return 0;
}
