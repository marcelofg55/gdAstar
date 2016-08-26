#ifndef __ASTAR_H__
#define __ASTAR_H__

#include <vector>
#include <utility>

#include "reference.h"

class gdAstar : public Reference {
	OBJ_TYPE(gdAstar, Reference);

protected:
	static void _bind_methods();

//	std::vector< std::pair<int, int> > map;

public:
	gdAstar();
	~gdAstar();

	void AddPoint(int x, int y);
	void ClearPoints();
	Vector2Array FindPath(int x0, int y0, int x1, int y1);
};

#endif
