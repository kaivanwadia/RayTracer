#include "cubeMap.h"
#include "ray.h"
#include "../ui/TraceUI.h"
extern TraceUI* traceUI;

Vec3d CubeMap::getColor(ray r) const {

	int axis, front, left, right, top, bottom;
	double u,v;
	Vec3d dir = r.getDirection();
	
	if (fabs(dir[0]) > fabs(dir[1]))
		if (fabs(dir[0]) > fabs(dir[2])) {
			axis = 0;
			bottom = 3;
			top = 2;
			if (dir[0] > 0.0) {
				front = 0;
				left = 4;
				right = 5;
			}
			else {
				front = 1;
				left = 5;
				right = 4;
			}
		}
		else {
			axis = 2;
			bottom = 3;
			top = 2;
			if (dir[2] > 0.0) {
				front = 5;
				left = 0;
				right = 1;
			}
			else {
				front = 4;
				left = 1;
				right = 0;
			}
		}
	else
		if (fabs(dir[1]) > fabs(dir[2])) {
			axis = 1;
			left = 1;
			right = 0;
			if (dir[1] > 0.0) {
				front = 2;
				bottom = 4;
				top = 5;
			}
			else {
				front = 3;
				bottom = 5;
				top = 4;
			}
		}
		else {
			axis = 2;
			bottom = 3;
			top = 2;
			if (dir[2] > 0.0) {
				front = 5;
				left = 0;
				right = 1;
			}
			else {
				front = 4;
				left = 1;
				right = 0;
			}
		}

	if (axis == 0) {
		u = dir[2]/dir[0];
		if (dir[0] > 0.0) v = dir[1]/dir[0];
		else v = -dir[1]/dir[0];
	}
	else if (axis == 1) {
		if (dir[1] > 0.0) u = dir[0]/dir[1];
		else u = -dir[0]/dir[1];
		v = dir[2]/dir[1];
	}
	else if (axis == 2) {
		u = -dir[0]/dir[2];
		if (dir[2] > 0.0) v = dir[1]/dir[2];
		else v = -dir[1]/dir[2];
	}

	u = (u + 1.0)/2.0;
	v = (v + 1.0)/2.0;

	int filterwidth = traceUI->getFilterWidth();
	if (r.type() != ray::VISIBILITY || filterwidth == 1) return tMap[front]->getMappedValue(Vec2d(u, v));
	int fw = (filterwidth + 1)/2 - 1;
	int rm = filterwidth - fw;
	int width = tMap[front]->getWidth();
	int height = tMap[front]->getHeight();
	double delu = 1.0/width;
	double delv = 1.0/height;
	int x = floor(0.5 + u * (width - 1));
	int y = floor(0.5 + v * (height - 1));
	double startu = delu * (x - fw);
	double startv = delv * (y - fw);
	double nextu = startu;
	double nextv = startv;
	double xslope = 1.0/(((double)filterwidth + 1) / 2.0 * delu);
	double yslope = 1.0/(((double)filterwidth + 1) / 2.0 * delv);
	double correct = 0.0;
	Vec3d thePixel = Vec3d(0.0, 0.0, 0.0);
	for (int jj = -fw; jj < rm; jj++) {
		for (int ii = -fw; ii < rm; ii++) {
			int xindex = x + ii;
			int yindex = y + jj;
			if (xindex < 0 && yindex < 0) continue;
			if (xindex >= width && yindex >= height) continue;
			if (xindex >= width && yindex < 0) continue;
			if (xindex < 0 && yindex >= height) continue;
			int theMap = front;
			if (yindex < 0) {
				theMap = bottom;
				if (axis == 0)
					if (dir[0] > 0) {
						yindex = height - 1 - xindex;
						xindex = width + jj;
					}
					else {
						yindex = xindex;
						xindex = -1 - jj;
					}
				else if (axis == 1)
					if (dir[1] > 0) yindex = height + jj;
					else {
						yindex = -1 - jj;
						xindex = width - 1 - xindex;
					}
				else if (axis == 2)
					if (dir[2] > 0) {
						yindex = -1 - jj;
						xindex = width - 1 - xindex;
					}
					else yindex = height + jj;
			}
			else if (yindex >= height) {
				theMap = top;
				if (axis == 0)
					if (dir[0] > 0) {
						yindex = xindex;
						xindex = width - jj;
					}
					else {
						yindex = height - 1 - xindex;
						xindex = jj - 1;
					}
				else if (axis == 1)
					if (dir[1] > 0)  {
						yindex = height - jj;
						xindex = width - 1 - xindex;
					}
					else yindex = jj - 1;
				else if (axis == 2)
					if (dir[2] > 0) {
						yindex =  height - jj;
						xindex = width - 1 - xindex;
					}
					else yindex = jj - 1;
			}
			else if (xindex < 0) {
				theMap = left;
				if (axis == 0  || axis == 2) xindex = width + ii;
				else if (axis == 1)
					if (dir[1] > 0)  {
						xindex = width - 1 - yindex;
						yindex = height + ii;
					}
					else {
						xindex = yindex;
						yindex = -1 - ii;
					}
			}
			else if (xindex >= width) {
				theMap = right;
				if (axis == 0  || axis == 2) xindex = ii - 1;
				else if (axis == 1)
					if (dir[1] > 0)  {
						xindex = yindex;
						yindex = height - ii;
					}
					else {
						xindex = width - 1 - yindex;
						yindex = ii - 1;
					}
			}
			double du = u - nextu;
			double dv = v - nextv;
			double weight = (1.0 - fabs(du * xslope)) * (1.0 - fabs(dv * yslope));
			nextu += delu;
			correct += weight;
			thePixel += tMap[theMap]->getPixelAt(xindex, yindex) * weight;
		}
		nextu = startu;
		nextv += delv;
	}
	thePixel /= correct;
	return thePixel;
}
