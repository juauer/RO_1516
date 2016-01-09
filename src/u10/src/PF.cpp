#include <algorithm>
#include <math.h>
#include "PF.h"

PF::PF(int _width, int _height) {
	width  = _width;
	height = _height;
	pf     = new float[width*height];
}

PF::~PF() {

}

void PF::addP(int x, int y, float r) {
	for(int i=0; i<height; ++i)
		for(int j=0; j<width; ++j) {
			float dx = x-j;
			float dy = y-i;
			float d  = sqrtf(dx*dx + dy*dy);

			if(d > fabs(r))
				continue;

			int k   = i*width + j;
			float v = r<0 ? 1.0f+d/r : -1.0f+d/r;

			if(fabs(v) > fabs(pf[k]))
				pf[k] = v;
		}
}

void PF::finalize() {
	for(int i=0; i<width*height; ++i)
		pf[i] = 0.5f + std::min(1.0f, std::max(-1.0f, pf[i])) / 2.0f;
}

std::vector<std::pair<int, int> > PF::findPath(int sx, int sy, int tx, int ty) {
	std::vector<std::pair<int, int> > path;
	int x = sx;
	int y = sy;

	while(x!=tx || y!=ty) {
		path.push_back(std::pair<int, int>(x, y));
		int k = y*width + x;
		pf[k] = std::max(0.0f, pf[k] - 0.01f);

		for(int i=(y==0?0:-1); i<(y==height-1?1:2); ++i)
			for(int j=(x==0?0:-1); j<(x==width-1?1:2); ++j) {
				int l = (y+i)*width + x + j;

				if(pf[l] > pf[k])
					k = l;
			}

		x = k % width;
		y = k / width;
	}

	return path;
}
