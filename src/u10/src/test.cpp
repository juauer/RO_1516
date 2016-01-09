#include <string.h>
#include "opencv2/opencv.hpp"
#include "PF.h"

const std::string path_captures = "./captures/";

int main(int, char**) {
	int s = 500;
	cv::RNG rng(time(NULL));
	PF pf(s, s);

	for(int i=0; i<8; ++i)
		pf.addP(rng.uniform(1, s), rng.uniform(1, s), rng.uniform(s/8, s/2));

	int sx = rng.uniform(1, s/4);
	int sy = rng.uniform(1, s/4);
	int tx = rng.uniform(3*s/4, s);
	int ty = rng.uniform(3*s/4, s);
	pf.addP(tx, ty, -1.5*s);
	pf.finalize();
	std::vector<std::pair<int, int> > path = pf.findPath(sx, sy, tx, ty);

	cv::Mat h(s, s, CV_32F, pf.pf);
	h *= 127.5;
	std::vector<cv::Mat> hsv;
	hsv.push_back(h);
	hsv.push_back(255*cv::Mat::ones(s, s, CV_32F));
	hsv.push_back(255*cv::Mat::ones(s, s, CV_32F));
	cv::Mat img;
	cv::merge(hsv, img);
	img.convertTo(img, CV_8UC3);
	cv::cvtColor(img, img, CV_HSV2BGR);
	cv::Scalar c(0, 0, 0);
	std::vector<std::pair<int, int> >::iterator p=path.begin();

	for(; p!=path.end(); ++p)
		cv::circle(img, cv::Point(p->first, p->second), 1, c, 2);

	cv::circle(img, cv::Point(sx, sy), 2, cv::Scalar(255, 0, 255), 2);
	cv::circle(img, cv::Point(tx, ty), 2, cv::Scalar(255, 255, 255), 2);
	cv::namedWindow("APF");
	cv::imshow("APF", img);
	cv::imwrite(path_captures+"capture_1-4.png", img);
	cv::waitKey(0);
	return 0;
}
