#include <cmath>
#include <limits>

#include "scene.h"
#include "light.h"
#include "../ui/TraceUI.h"

using namespace std;

bool Geometry::intersect(ray& r, isect& i) const {
	double tmin, tmax;
	if (hasBoundingBoxCapability() && !(bounds.intersect(r, tmin, tmax))) return false;
	// Transform the ray into the object's local coordinate space
	Vec3d pos = transform->globalToLocalCoords(r.p);
	Vec3d dir = transform->globalToLocalCoords(r.p + r.d) - pos;
	double length = dir.length();
	dir /= length;
	Vec3d Wpos = r.p;
	Vec3d Wdir = r.d;
	r.p = pos;
	r.d = dir;
	bool rtrn = false;
	if (intersectLocal(r, i))
	{
		// Transform the intersection point & normal returned back into global space.
		i.N = transform->localToGlobalCoordsNormal(i.N);
		i.t /= length;
		rtrn = true;
	}
	r.p = Wpos;
	r.d = Wdir;
	return rtrn;
}

bool Geometry::hasBoundingBoxCapability() const {
	// by default, primitives do not have to specify a bounding box.
	// If this method returns true for a primitive, then either the ComputeBoundingBox() or
    // the ComputeLocalBoundingBox() method must be implemented.

	// If no bounding box capability is supported for an object, that object will
	// be checked against every single ray drawn.  This should be avoided whenever possible,
	// but this possibility exists so that new primitives will not have to have bounding
	// boxes implemented for them.
	return false;
}

Scene::~Scene() {
    giter g;
    liter l;
    tmap::iterator t;
    for( g = objects.begin(); g != objects.end(); ++g ) delete (*g);
    for( l = lights.begin(); l != lights.end(); ++l ) delete (*l);
    for( t = textureCache.begin(); t != textureCache.end(); t++ ) delete (*t).second;
}

// Get any intersection with an object.  Return information about the 
// intersection through the reference parameter.
bool Scene::intersect(ray& r, isect& i) const {
	double tmin = 0.0;
	double tmax = 0.0;
	bool have_one = false;
	typedef vector<Geometry*>::const_iterator iter;
	for(iter j = objects.begin(); j != objects.end(); ++j) {
		isect cur;
		if( (*j)->intersect(r, cur) ) {
			if(!have_one || (cur.t < i.t)) {
				i = cur;
				have_one = true;
			}
		}
	}
	if(!have_one) i.setT(1000.0);
	// if debugging,
	if (TraceUI::m_debug) intersectCache.push_back(std::make_pair(new ray(r), new isect(i)));
	return have_one;
}

TextureMap* Scene::getTexture(string name) {
	tmap::const_iterator itr = textureCache.find(name);
	if(itr == textureCache.end()) {
		textureCache[name] = new TextureMap(name);
		return textureCache[name];
	} else return (*itr).second;
}

void Scene::buildKdTree(int depth, int leafSize) {
	this->kdtree = new KdTree<Geometry>(true);
	for (auto objIter = beginBoundedObjects(); objIter != endBoundedObjects(); objIter++)
	{
		kdtree->addObject(*objIter);
		kdtree->setBoundingBox(this->bounds());
	}
	buildMainKdTree(kdtree, depth-1, leafSize);
}

void Scene::buildMainKdTree(KdTree<Geometry>* kdtree, int depth, int leafSize)
{
	if (depth == 0)
	{
		return;
	}
	double minCost = numeric_limits<double>::max();
	BoundingBox minBoundingBox;
	int minAxis;
	for (int i = 0; i<kdtree->noOfObjects(); i++)
	{
		Geometry* object = kdtree->getObject(i);
		for (int planeNo = 0; planeNo < 6; planeNo++)
		{
			double leftVolume = 0;
			double rightVolume = 0;
			double boxVolume = kdtree->getBoundingBox().volume();
			double noLeft = 0;
			double noRight = 0;
			Vec3d point;
			if (planeNo > 2)
			{
				point = object->getBoundingBox().getMax();
			}
			else
			{
				point = object->getBoundingBox().getMin();
			}
			int axis = planeNo%3;
			Vec3d plane(0,0,0);
			plane[axis] = point[axis];
			Vec3d bMax = kdtree->getBoundingBox().getMax();
			Vec3d bMin = kdtree->getBoundingBox().getMin();
			bMax[axis] = plane[axis];
			bMin[axis] = plane[axis];
			BoundingBox planeBB(bMin, bMax);
			for (int k = 0; k<kdtree->noOfObjects(); k++)
			{
				Geometry* object2 = kdtree->getObject(k);
				BoundingBox obj2bb = object2->getBoundingBox();
				if (planeBB.intersects(obj2bb))
				{
					// Add cost to both sides
					leftVolume += obj2bb.volume();
					rightVolume += obj2bb.volume();
					noLeft++;
					noRight++;
				}
				else if (obj2bb.getMax()[axis] < planeBB.getMin()[axis])
				{
					// Add to left (a)
					leftVolume += obj2bb.volume();
					noLeft++;
				}
				else
				{
					// Add to right (b)
					rightVolume += obj2bb.volume();
					noRight++;
				}
			}
			double pLeft = leftVolume/boxVolume;
			double pRight = rightVolume/boxVolume;
			double cost = 1 + pLeft*noLeft*(80) + pRight*noRight*(80);
			if (cost < minCost)
			{
				minCost = cost;
				minBoundingBox = planeBB;
				minAxis = axis;
			}
		}
	}
	KdTree<Geometry>* left = new KdTree<Geometry>();
	KdTree<Geometry>* right = new KdTree<Geometry>();
	for (int i = 0; i<kdtree->noOfObjects(); i++)
	{
		Geometry* object = kdtree->getObject(i);
		BoundingBox objBB = object->getBoundingBox();
		if (minBoundingBox.intersects(objBB))
		{
			left->addObject(object);
			left->getBoundingBox().merge(objBB);
			right->addObject(object);
			right->getBoundingBox().merge(objBB);
		}
		else if (objBB.getMax()[minAxis] < minBoundingBox.getMin()[minAxis])
		{
			left->addObject(object);
			left->getBoundingBox().merge(objBB);
		}
		else
		{
			right->addObject(object);
			right->getBoundingBox().merge(objBB);
		}
	}
	kdtree->setLeft(left);
	kdtree->setRight(right);
	if (left->noOfObjects() > leafSize)
	{
		buildMainKdTree(left, depth-1, leafSize);
	}
	if (right->noOfObjects() > leafSize)
	{
		buildMainKdTree(right, depth-1, leafSize);
	}
}