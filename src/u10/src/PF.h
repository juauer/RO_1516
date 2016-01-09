#ifndef PF_H_
#define PF_H_

#include <vector>

class PF {
public:
	int width, height;
	float *pf;
	PF(int width, int height);
	~PF();
	void addP(int x, int y, float r);
	void finalize();
	std::vector<std::pair<int, int> > findPath(int sx, int sy, int tx, int ty);
};

#endif /* PF_H_ */
