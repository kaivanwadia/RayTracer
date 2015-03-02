#include <cmath>
#include <limits>
#include <stack>

#include "scene.h"
#include "light.h"
#include "../ui/TraceUI.h"
#include "../SceneObjects/trimesh.h"

using namespace std;

int Geometry::idGen = 1;
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

void Scene::intersectKdTree(ray& r, isect& i, KdTree<Geometry>* currentNode, bool& have_one, double tMin, double tMax) const
{
	stack<StackElement> kdTreeStack;
	struct StackElement rootElement = StackElement(currentNode, tMin, tMax);
	kdTreeStack.push(rootElement);
	while (!kdTreeStack.empty())
	{
		struct StackElement currElem = kdTreeStack.top();
		kdTreeStack.pop();
		if ((currElem.currNode->left == nullptr) && (currElem.currNode->right == nullptr))
		{
			// cout<<"LEAF NODE REACHED\n";
			for (cgiter obj = currElem.currNode->objectsVector.begin(); obj != currElem.currNode->objectsVector.end(); obj++)
			{
				isect cur;
				if ((*obj)->isTrimesh())
				{
					Trimesh* triMesh = (Trimesh*)(*obj);
					double tTriMin,tTriMax;
					tTriMin = tTriMax = 0.0;
					bool triMeshHit = triMesh->kdtreeRoot->bb.intersect(r, tTriMin, tTriMax);
					intersectKdTree(r, i, triMesh->kdtreeRoot, have_one, tTriMin, tTriMax);
				}
				else
				{
					if ((*obj)->intersect(r,cur))
					{
						if (!have_one || cur.t < i.t) {
							i = cur;
							have_one = true;
						}
					}
				}
			}
		}
		else
		{
			KdTree<Geometry> *nearNode,*farNode;
			if (r.p[currElem.currNode->dimension] < currElem.currNode->splittingBB.getMin()[currElem.currNode->dimension])
			{
				nearNode = currElem.currNode->left;
				farNode = currElem.currNode->right;
			}
			else
			{
				nearNode = currElem.currNode->right;
				farNode = currElem.currNode->left;
			}
			double tStar = ((currElem.currNode->splittingBB.getMin()[currElem.currNode->dimension] - r.p[currElem.currNode->dimension])/r.d[currElem.currNode->dimension]);
			if (tStar > currElem.tMax || tStar < 0)
			{
				kdTreeStack.push(StackElement(nearNode, tMin, tMax));
				//intersectKdTree(r, i, nearNode, have_one, tMin, tMax);
			}
			else if (tStar < currElem.tMin)
			{
				kdTreeStack.push(StackElement(farNode, tMin, tMax));
				// intersectKdTree(r, i, farNode, have_one, tMin, tMax);
			}
			else if (currElem.tMin <= tStar && tStar <= currElem.tMax)
			{
				kdTreeStack.push(StackElement(nearNode, tMin, tStar));
				kdTreeStack.push(StackElement(farNode, tStar, tMax));
				// intersectKdTree(r, i, nearNode, have_one, tMin, tStar);
				// intersectKdTree(r, i, farNode, have_one, tStar, tMax);
			}
		}

	}
	return;
}

// bool Scene::intersectKdTreeMain(ray& r, isect& i) const {
// 	bool have_one = false;
// 	double tMin,tMax;
// 	tMin = tMax = 0.0;
// 	bool sceneHit = this->kdtreeRoot->bb.intersect(r, tMin, tMax);
// 	intersectKdTree(r, i, this->kdtreeRoot, have_one, tMin, tMax);
// 	if(!have_one) i.setT(1000.0);
// 	// if debugging,
// 	if (TraceUI::m_debug) intersectCache.push_back(std::make_pair(new ray(r), new isect(i)));
// 	return have_one;
// }

// Get any intersection with an object.  Return information about the 
// intersection through the reference parameter.
bool Scene::intersect(ray& r, isect& i) const {
	bool have_one = false;
	if (this->useKdTree)
	{
		double tMin,tMax;
		tMin = tMax = 0.0;
		bool sceneHit = this->kdtreeRoot->bb.intersect(r, tMin, tMax);
		intersectKdTree(r, i, this->kdtreeRoot, have_one, tMin, tMax);
	}
	else
	{
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

void Scene::printKdTree(KdTree<Geometry>* root) {
	vector<KdTree<Geometry>*> currentLevel;
	vector<KdTree<Geometry>*> nextLevel;
	nextLevel.push_back(root);
	int levelNo = 0;
	while (nextLevel.size() > 0)
	{
		cout << "Level : "<<levelNo << "\n";
		currentLevel = nextLevel;
		nextLevel = vector<KdTree<Geometry>*>();
		for (int i = 0; i < currentLevel.size(); i++)
		{
			cout << currentLevel[i]->noOfObjects()<<"("<<currentLevel[i]->bb.area()<<")\t";
			// for (int j = 0; j < currentLevel[i]->noOfObjects(); j++)
			// {
			// 	cout<<currentLevel[i]->objectsVector[j]->objectID<<",";
			// }
			// cout<<")"<<currentLevel[i]->bb.getMin() <<" "<<currentLevel[i]->bb.getMax()<<"\t";
			if (i % 2 == 1)
			{
				cout << "\t\t";
			}
			if (currentLevel[i]->left != nullptr)
			{
				nextLevel.push_back(currentLevel[i]->left);
			}
			if (currentLevel[i]->right != nullptr)
			{
				nextLevel.push_back(currentLevel[i]->right);
			}
		}
		levelNo++;
		cout << "\n";
	}
}

bool yzPlaneCompareFunction(Geometry* first, Geometry* second)
{
	BoundingBox firstBB = first->getBoundingBox();
	BoundingBox secondBB = second->getBoundingBox();
	double firstX = firstBB.getMin()[0];
	double secondX = secondBB.getMin()[0];
	return (firstX < secondX);
}

bool xzPlaneCompareFunction(Geometry* first, Geometry* second)
{
	BoundingBox firstBB = first->getBoundingBox();
	BoundingBox secondBB = second->getBoundingBox();
	double firstY = firstBB.getMin()[1];
	double secondY = secondBB.getMin()[1];
	return (firstY < secondY);
}

bool xyPlaneCompareFunction(Geometry* first, Geometry* second)
{
	BoundingBox firstBB = first->getBoundingBox();
	BoundingBox secondBB = second->getBoundingBox();
	double firstZ = firstBB.getMin()[2];
	double secondZ = secondBB.getMin()[2];
	return (firstZ < secondZ);
}

void Scene::buildKdTree(int depth, int leafSize) {
	this->kdTreeDepth = depth;
	this->kdTreeLeafSize = leafSize;
	this->kdtreeRoot = new KdTree<Geometry>(true);
	for (auto objIter = beginObjects(); objIter != endObjects(); objIter++)
	{
		// !(*objIter)->isTrimesh() && 
		if ((*objIter)->hasBoundingBoxCapability())
		{
			kdtreeRoot->objectsVector.push_back(*objIter);
		}
		// else if ((*objIter)->isTrimesh() && (*objIter)->hasBoundingBoxCapability())
		// {
		// 	Trimesh *triMesh = (Trimesh*)(*objIter);
		// 	for (int i = 0; i < triMesh->faces.size(); i++)
		// 	{
		// 		kdtreeRoot->objectsVector.push_back(triMesh->faces[i]);
		// 	}
		// }
	}
	// cout<<"Number of Objects: "<<kdtreeRoot->noOfObjects()<<endl;
	kdtreeRoot->bb = this->bounds();
    std::vector<Geometry*> yzPlaneOrder;
    std::vector<Geometry*> xzPlaneOrder;
    std::vector<Geometry*> xyPlaneOrder;
    for(int i = 0; i < kdtreeRoot->noOfObjects(); i++) {
        yzPlaneOrder.push_back(kdtreeRoot->objectsVector[i]);
        xzPlaneOrder.push_back(kdtreeRoot->objectsVector[i]);
        xyPlaneOrder.push_back(kdtreeRoot->objectsVector[i]);
    }
    sort(yzPlaneOrder.begin(), yzPlaneOrder.end(), yzPlaneCompareFunction);
    sort(xzPlaneOrder.begin(), xzPlaneOrder.end(), xzPlaneCompareFunction);
    sort(xyPlaneOrder.begin(), xyPlaneOrder.end(), xyPlaneCompareFunction);

    vector<vector<Geometry*>> orderedPlanes;
    orderedPlanes.push_back(yzPlaneOrder);
    orderedPlanes.push_back(xzPlaneOrder);
    orderedPlanes.push_back(xyPlaneOrder);

    // cout << "X Order\n";
    // for(auto ii : yzPlaneOrder ) {
    //   cout << "Min: "<<ii->getBoundingBox().getMin() << "\n";
    //   cout << "Max: "<<ii->getBoundingBox().getMax() << "\n";
    // }
    // cout << "Y Order\n";
    // for(auto ii : xzPlaneOrder ) {
    //   cout << "Min: "<<ii->getBoundingBox().getMin() << "\n";
    //   cout << "Max: "<<ii->getBoundingBox().getMax() << "\n";
    // }
    // cout << "Z Order\n";
    // for(auto ii : xyPlaneOrder ) {
    //   cout << "Min: "<<ii->getBoundingBox().getMin() << "\n";
    //   cout << "Max: "<<ii->getBoundingBox().getMax() << "\n";
    // }
    // cout<<"Scene BB : "<<this->kdtreeRoot->bb.getMin() << "||" << this->kdtreeRoot->bb.getMax()<<endl;
	buildMainKdTree(kdtreeRoot, depth, leafSize, orderedPlanes);
	// cout<<"MAIN KD TREE"<<endl;
	printKdTree(this->kdtreeRoot);
	// cout<<"END MAIN KD TREE"<<endl;
}

void Scene::buildMainKdTree(KdTree<Geometry>* kdNode, int depth, int leafSize, vector<vector<Geometry*>> orderedPlanes)
{
	if (depth == 0)
	{
		return;
	}
	else
	{
		for (int i = 0; i<kdNode->noOfObjects(); i++)
		{
			if (kdNode->objectsVector[i]->isTrimesh())
			{
				Trimesh *triMesh = (Trimesh*)(kdNode->objectsVector[i]);
				if (!triMesh->kdTreeBuilt())
				{
					buildTrimeshKdTree(triMesh, this->kdTreeDepth, this->kdTreeLeafSize);
				}
			}
		}
	}
	// cout<<"SIZE X: "<<orderedPlanes[0].size()<<endl;
	// cout<<"SIZE Y: "<<orderedPlanes[1].size()<<endl;
	// cout<<"SIZE Z: "<<orderedPlanes[2].size()<<endl;
	double finalCost = numeric_limits<double>::max();
	BoundingBox finalBB;
	int finalDimension;
	Vec3d finalPoint;
	Geometry* finalObject;
	bool finalMinMax;
	for (int dimen = 0; dimen < 3; dimen++)
	{
		for (vector<Geometry*>::const_iterator ii = orderedPlanes[dimen].begin(); ii != orderedPlanes[dimen].end(); ii++)
		{
			BoundingBox iBoundingBox = (*ii)->getBoundingBox();
			// cout<<"Box : "<<iBoundingBox.getMin() <<"||"<<iBoundingBox.getMax()<<endl;
			double minValue = iBoundingBox.getMin()[dimen];
			double maxValue = iBoundingBox.getMax()[dimen];
			double minLeftArea = 0;
			double minRightArea = 0;
			double minLeftObjects = 0;
			double minRightObjects = 0;
			double maxLeftArea = 0;
			double maxRightArea = 0;
			double maxLeftObjects = 0;
			double maxRightObjects = 0;
			for (vector<Geometry*>::const_iterator jj = orderedPlanes[dimen].begin(); jj != orderedPlanes[dimen].end(); jj++)
			{
				// If object is same add to minimum plane and maximum plane calculation
				if (*ii == *jj)
				{
					minRightArea = minRightArea + iBoundingBox.area();
					minRightObjects++;
					maxLeftArea = maxLeftArea + iBoundingBox.area();
					maxLeftObjects++;
					continue;
				}
				BoundingBox jBoundingBox = (*jj)->getBoundingBox();
				// Calculations for the minimum plane of the object ii
				if (jBoundingBox.getMin()[dimen] < minValue && jBoundingBox.getMax()[dimen] <= minValue)
				{
					minLeftArea = minLeftArea + jBoundingBox.area();
					minLeftObjects++;
				}
				else if (jBoundingBox.getMin()[dimen] >= minValue)
				{
					minRightArea = minRightArea + jBoundingBox.area();
					minRightObjects++;
				}
				else
				{
					minLeftArea = minLeftArea + jBoundingBox.area();
					minLeftObjects++;
					minRightArea = minRightArea + jBoundingBox.area();
					minRightObjects++;
				}
				// Calculations for the maximum plane of the object ii
				if (jBoundingBox.getMin()[dimen] < maxValue && jBoundingBox.getMax()[dimen] <= maxValue)
				{
					maxLeftArea = maxLeftArea + jBoundingBox.area();
					maxLeftObjects++;
				}
				else if (jBoundingBox.getMin()[dimen] >= maxValue)
				{
					maxRightArea = maxRightArea + jBoundingBox.area();
					maxRightObjects++;
				}
				else
				{
					maxLeftArea = maxLeftArea + jBoundingBox.area();
					maxLeftObjects++;
					maxRightArea = maxRightArea + jBoundingBox.area();
					maxRightObjects++;
				}
			}
			double minPlaneCost = 1 + (minLeftArea)*minLeftObjects*80 + (minRightArea)*minRightObjects*80;
			double maxPlaneCost = 1 + (maxLeftArea)*maxLeftObjects*80 + (maxRightArea)*maxRightObjects*80;
			if (minPlaneCost < finalCost)
			{
				finalCost = minPlaneCost;
				finalDimension = dimen;
				finalPoint = iBoundingBox.getMin();
				finalObject = *ii;
				finalMinMax = false;
			}
			if (maxPlaneCost < finalCost)
			{
				finalCost = maxPlaneCost;
				finalDimension = dimen;
				finalPoint = iBoundingBox.getMax();
				finalObject = *ii;
				finalMinMax = true;
			}
		}
	}
	// cout << "Final Cost : " << finalCost << "\n";
	// cout << "Final Dimension : "<<finalDimension<<" \n";
	// cout << "Final Point : "<<finalPoint<<"\n";
	KdTree<Geometry>* leftChild = new KdTree<Geometry>();
	KdTree<Geometry>* rightChild = new KdTree<Geometry>();
	vector<vector<Geometry*>> leftOrderedPlanes(3);
	vector<vector<Geometry*>> rightOrderedPlanes(3);
	leftOrderedPlanes.push_back(vector<Geometry*>());
	leftOrderedPlanes.push_back(vector<Geometry*>());
	leftOrderedPlanes.push_back(vector<Geometry*>());
	rightOrderedPlanes.push_back(vector<Geometry*>());
	rightOrderedPlanes.push_back(vector<Geometry*>());
	rightOrderedPlanes.push_back(vector<Geometry*>());

	kdNode->splittingPlane[finalDimension] = finalPoint[finalDimension];
	kdNode->dimension = finalDimension;
	// Vec3d minPoint = kdNode->bb.getMin();
	// Vec3d maxPoint = kdNode->bb.getMin();
	Vec3d minPoint = Vec3d(-1.0e308, -1.0e308, -1.0e308);
	Vec3d maxPoint = Vec3d(1.0e308, 1.0e308, 1.0e308);
	minPoint[finalDimension] = finalPoint[finalDimension];
	maxPoint[finalDimension] = finalPoint[finalDimension];
	kdNode->splittingBB = BoundingBox(minPoint, maxPoint);
	for (int dimen = 0; dimen < 3; dimen++)
	{
		for (vector<Geometry*>::const_iterator ii = orderedPlanes[dimen].begin(); ii != orderedPlanes[dimen].end(); ii++)
		{
			BoundingBox iBoundingBox = (*ii)->getBoundingBox();
			// if (*ii == finalObject)
			// {
			// 	if (!finalMinMax)
			// 	{
			// 		leftChild->objectsVector.push_back(*ii);
			// 		leftChild->bb.merge(iBoundingBox);
			// 		leftOrderedPlanes[dimen].push_back(*ii);
			// 	}
			// 	else
			// 	{
			// 		rightChild->objectsVector.push_back(*ii);
			// 		rightChild->bb.merge(iBoundingBox);
			// 		rightOrderedPlanes[dimen].push_back(*ii);
			// 	}
			// 	cout<<"Here\n";
			// 	continue;
			// }
			if (iBoundingBox.getMin()[finalDimension] < finalPoint[finalDimension] && iBoundingBox.getMax()[finalDimension] <= finalPoint[finalDimension])
			{
				if (dimen == finalDimension)
				{
					leftChild->objectsVector.push_back(*ii);
					leftChild->bb.merge(iBoundingBox);
				}
				leftOrderedPlanes[dimen].push_back(*ii);
			}
			else if (iBoundingBox.getMin()[finalDimension] > finalPoint[finalDimension])
			{
				if (dimen == finalDimension)
				{
					rightChild->objectsVector.push_back(*ii);
					rightChild->bb.merge(iBoundingBox);
				}
				rightOrderedPlanes[dimen].push_back(*ii);
			}
			else
			{
				if (dimen == finalDimension)
				{
					leftChild->objectsVector.push_back(*ii);
					leftChild->bb.merge(iBoundingBox);
					rightChild->objectsVector.push_back(*ii);
					rightChild->bb.merge(iBoundingBox);
				}
				leftOrderedPlanes[dimen].push_back(*ii);
				rightOrderedPlanes[dimen].push_back(*ii);
			}
		}
	}
	kdNode->setLeft(leftChild);
	kdNode->setRight(rightChild);
	if (kdNode->getLeft()->noOfObjects() > leafSize)
	{
		buildMainKdTree(kdNode->left, depth-1, leafSize, leftOrderedPlanes);
	}
	else
	{
		for (int i = 0; i<kdNode->getLeft()->noOfObjects(); i++)
		{
			if (kdNode->getLeft()->objectsVector[i]->isTrimesh())
			{
				Trimesh *triMesh = (Trimesh*)(kdNode->getLeft()->objectsVector[i]);
				if (!triMesh->kdTreeBuilt())
				{
					buildTrimeshKdTree(triMesh, this->kdTreeDepth, this->kdTreeLeafSize);
				}
			}
		}
	}
	if (kdNode->getRight()->noOfObjects() > leafSize)
	{
		buildMainKdTree(kdNode->right, depth-1, leafSize, rightOrderedPlanes);
	}
	else
	{
		for (int i = 0; i<kdNode->getRight()->noOfObjects(); i++)
		{
			if (kdNode->getRight()->objectsVector[i]->isTrimesh())
			{
				Trimesh *triMesh = (Trimesh*)(kdNode->getRight()->objectsVector[i]);
				if (!triMesh->kdTreeBuilt())
				{
					buildTrimeshKdTree(triMesh, this->kdTreeDepth, this->kdTreeLeafSize);
				}
			}
		}
	}
}

void Scene::buildTrimeshKdTree(Geometry* triM, int depth, int leafSize)
{
	Trimesh *triMesh = (Trimesh*)(triM);
	triMesh->kdtreeRoot = new KdTree<Geometry>(true);
	for (int i = 0; i < triMesh->faces.size(); i++)
	{
		triMesh->kdtreeRoot->objectsVector.push_back(triMesh->faces[i]);
	}
	// cout<<"Number of Objects: "<<kdtreeRoot->noOfObjects()<<endl;
	triMesh->kdtreeRoot->bb = triMesh->getBoundingBox();
    std::vector<Geometry*> yzPlaneOrder;
    std::vector<Geometry*> xzPlaneOrder;
    std::vector<Geometry*> xyPlaneOrder;
    for(int i = 0; i < triMesh->kdtreeRoot->noOfObjects(); i++) {
        yzPlaneOrder.push_back(triMesh->kdtreeRoot->objectsVector[i]);
        xzPlaneOrder.push_back(triMesh->kdtreeRoot->objectsVector[i]);
        xyPlaneOrder.push_back(triMesh->kdtreeRoot->objectsVector[i]);
    }
    sort(yzPlaneOrder.begin(), yzPlaneOrder.end(), yzPlaneCompareFunction);
    sort(xzPlaneOrder.begin(), xzPlaneOrder.end(), xzPlaneCompareFunction);
    sort(xyPlaneOrder.begin(), xyPlaneOrder.end(), xyPlaneCompareFunction);

    vector<vector<Geometry*>> orderedPlanes;
    orderedPlanes.push_back(yzPlaneOrder);
    orderedPlanes.push_back(xzPlaneOrder);
    orderedPlanes.push_back(xyPlaneOrder);
	buildMainKdTree(triMesh->kdtreeRoot, depth, leafSize, orderedPlanes);
	cout<<"START TRIMESH TREE"<<endl;
	printKdTree(triMesh->kdtreeRoot);
	cout<<"END TRIMESH TREE"<<endl;
}