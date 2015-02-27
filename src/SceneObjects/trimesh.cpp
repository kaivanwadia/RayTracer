#include <cmath>
#include <float.h>
#include <algorithm>
#include <assert.h>
#include "trimesh.h"
#include "../ui/TraceUI.h"
extern TraceUI* traceUI;

using namespace std;

Trimesh::~Trimesh()
{
	for( Materials::iterator i = materials.begin(); i != materials.end(); ++i )
		delete *i;
}

// must add vertices, normals, and materials IN ORDER
void Trimesh::addVertex( const Vec3d &v )
{
    vertices.push_back( v );
}

void Trimesh::addMaterial( Material *m )
{
    materials.push_back( m );
}

void Trimesh::addNormal( const Vec3d &n )
{
    normals.push_back( n );
}

// Returns false if the vertices a,b,c don't all exist
bool Trimesh::addFace( int a, int b, int c )
{
    int vcnt = vertices.size();

    if( a >= vcnt || b >= vcnt || c >= vcnt ) return false;

    TrimeshFace *newFace = new TrimeshFace( scene, new Material(*this->material), this, a, b, c );
    newFace->setTransform(this->transform);
    if (!newFace->degen) faces.push_back( newFace );


    // Don't add faces to the scene's object list so we can cull by bounding box
    // scene->add(newFace);
    return true;
}

char* Trimesh::doubleCheck()
// Check to make sure that if we have per-vertex materials or normals
// they are the right number.
{
    if( !materials.empty() && materials.size() != vertices.size() )
        return "Bad Trimesh: Wrong number of materials.";
    if( !normals.empty() && normals.size() != vertices.size() )
        return "Bad Trimesh: Wrong number of normals.";

    return 0;
}

bool Trimesh::intersectLocal(ray& r, isect& i) const
{
	double tmin = 0.0;
	double tmax = 0.0;
	typedef Faces::const_iterator iter;
	bool have_one = false;
	for( iter j = faces.begin(); j != faces.end(); ++j )
	  {
	    isect cur;
	    if( (*j)->intersectLocal( r, cur ) )
	      {
		if( !have_one || (cur.t < i.t) )
		  {
		    i = cur;
		    have_one = true;
		  }
	      }
	  }
	if( !have_one ) i.setT(1000.0);
	return have_one;
}

bool TrimeshFace::intersect(ray& r, isect& i) const {
  return intersectLocal(r, i);
}

// Intersect ray r with the triangle abc.  If it hits returns true,
// and put the parameter in t and the barycentric coordinates of the
// intersection in u (alpha) and v (beta).
bool TrimeshFace::intersectLocal(ray& r, isect& i) const
{

    const Vec3d& a = parent->vertices[ids[0]];
    const Vec3d& b = parent->vertices[ids[1]];
    const Vec3d& c = parent->vertices[ids[2]];

    double dConstant = -(a*normal);
    if (normal*r.d == 0)
    {
        return false;
    }
    double rayT = -(normal*r.p + dConstant)/(normal*r.d);
    if (rayT < RAY_EPSILON)
    {
        return false;
    }
    Vec3d p = r.p + rayT*r.d;
    Vec3d aUVCoord;
    Vec3d bUVCoord;
    Vec3d cUVCoord;
    Vec3d pUVCoord;
    int x,y;
    Vec3d normalAbs(abs(normal[0]),abs(normal[1]),abs(normal[2]));
    if (normalAbs[0] >= normalAbs[1] && normalAbs[0] >= normalAbs[2])
    {
        x = 1;
        y = 2;
    }
    else if (normalAbs[1] >= normalAbs[0] && normalAbs[1] >= normalAbs[2])
    {
        x = 0;
        y = 2;
    }
    else
    {
        x = 0;
        y = 1;
    }
    aUVCoord[0] = a[x];
    aUVCoord[1] = a[y];
    bUVCoord[0] = b[x];
    bUVCoord[1] = b[y];
    cUVCoord[0] = c[x];
    cUVCoord[1] = c[y];
    pUVCoord[0] = p[x];
    pUVCoord[1] = p[y];
    double ABCarea = ((bUVCoord - aUVCoord)^(cUVCoord - aUVCoord)).length()/2;
    double PBCarea = ((bUVCoord - pUVCoord)^(cUVCoord - pUVCoord)).length()/2;
    double APCarea = ((pUVCoord - aUVCoord)^(cUVCoord - aUVCoord)).length()/2;
    double ABParea = ((bUVCoord - aUVCoord)^(pUVCoord - aUVCoord)).length()/2;
    Vec3d baryCoord;
    baryCoord[0] = PBCarea/ABCarea;
    baryCoord[1] = APCarea/ABCarea;
    baryCoord[2] = ABParea/ABCarea;
    double total = baryCoord[0] + baryCoord[1] + baryCoord[2];
    if (total<=(1+RAY_EPSILON) && total>=(1-RAY_EPSILON))
    {
        i.t = rayT;
        if (parent->vertNorms)
        {
            Vec3d normalA = parent->normals[ids[0]];
            Vec3d normalB = parent->normals[ids[1]];
            Vec3d normalC = parent->normals[ids[2]];
            Vec3d normalIntersect = (baryCoord[0]*normalA) + (baryCoord[1]*normalB) + (baryCoord[2]*normalC);
            i.setN(normalIntersect);
        }
        else
        {
            i.setN(normal);
        }
        i.N.normalize();
        i.setBary(baryCoord);
        i.setUVCoordinates(Vec2d(baryCoord));
        if(parent->materials.size()>0)
        {
            Material aMaterial = *(parent->materials[ids[0]]);
            Material bMaterial = *(parent->materials[ids[1]]);
            Material cMaterial = *(parent->materials[ids[2]]);
            Material pMaterial;
            pMaterial += (baryCoord[0]*aMaterial);
            pMaterial += (baryCoord[1]*bMaterial);
            pMaterial += (baryCoord[2]*cMaterial);
            i.setMaterial(pMaterial);
        }
        else
        {
            i.setMaterial(parent->getMaterial());
        }
        i.setObject(this);
        return true;
    }
    return false;
}

void Trimesh::generateNormals()
// Once you've loaded all the verts and faces, we can generate per
// vertex normals by averaging the normals of the neighboring faces.
{
    int cnt = vertices.size();
    normals.resize( cnt );
    int *numFaces = new int[ cnt ]; // the number of faces assoc. with each vertex
    memset( numFaces, 0, sizeof(int)*cnt );
    
    for( Faces::iterator fi = faces.begin(); fi != faces.end(); ++fi )
    {
		Vec3d faceNormal = (**fi).getNormal();
        
        for( int i = 0; i < 3; ++i )
        {
            normals[(**fi)[i]] += faceNormal;
            ++numFaces[(**fi)[i]];
        }
    }

    for( int i = 0; i < cnt; ++i )
    {
        if( numFaces[i] )
            normals[i]  /= numFaces[i];
    }

    delete [] numFaces;
    vertNorms = true;
}
