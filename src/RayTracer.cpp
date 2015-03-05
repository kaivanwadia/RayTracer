// The main ray tracer.

#pragma warning (disable: 4786)

#include "RayTracer.h"
#include "scene/light.h"
#include "scene/material.h"
#include "scene/ray.h"

#include "parser/Tokenizer.h"
#include "parser/Parser.h"

#include "ui/TraceUI.h"
#include <cmath>
#include <algorithm>

extern TraceUI* traceUI;

#include <iostream>
#include <fstream>

using namespace std;

// Use this variable to decide if you want to print out
// debugging messages.  Gets set in the "trace single ray" mode
// in TraceGLWindow, for example.
bool debugMode = false;

// Trace a top-level ray through pixel(i,j), i.e. normalized window coordinates (x,y),
// through the projection plane, and out into the scene.  All we do is
// enter the main ray-tracing method, getting things started by plugging
// in an initial ray weight of (0.0,0.0,0.0) and an initial recursion depth of 0.

Vec3d RayTracer::trace(double x, double y)
{
  // Clear out the ray cache in the scene for debugging purposes,
  if (TraceUI::m_debug) scene->intersectCache.clear();
  ray r(Vec3d(0,0,0), Vec3d(0,0,0), ray::VISIBILITY);
  scene->getCamera().rayThrough(x,y,r);
  Vec3d ret = traceRay(r, traceUI->getDepth());
  ret.clamp();
  return ret;
}

Vec3d RayTracer::tracePixel(int i, int j)
{
	Vec3d col(0,0,0);

	if( ! sceneLoaded() ) return col;

	double x = double(i)/double(buffer_width);
	double y = double(j)/double(buffer_height);

	unsigned char *pixel = buffer + ( i + j * buffer_width ) * 3;

	col = trace(x, y);

	pixel[0] = (int)( 255.0 * col[0]);
	pixel[1] = (int)( 255.0 * col[1]);
	pixel[2] = (int)( 255.0 * col[2]);
	return col;
}

Vec3d RayTracer::tracePixelAntiAlias(int i, int j)
{
	Vec3d col(0,0,0);

	if( ! sceneLoaded() ) return col;

	srand((unsigned)time(NULL));
	double x = double(i)/double(buffer_width);
	double y = double(j)/double(buffer_height);

	unsigned char *pixel = buffer + ( i + j * buffer_width ) * 3;

	if (traceUI->antiAliasingWhite())
	{
		pixel[0] = (int)( 255.0 * 1);
		pixel[1] = (int)( 255.0 * 1);
		pixel[2] = (int)( 255.0 * 1);
		return col;
	}

    double deltaX = (1.0/double(buffer_width))/traceUI->m_nPixelSamples;
    double deltaY = (1.0/double(buffer_height))/traceUI->m_nPixelSamples;

    for(int subSampleCol = 0; subSampleCol < traceUI->m_nPixelSamples; subSampleCol++)
	{
		for(int subSampleRow = 0; subSampleRow < traceUI->m_nPixelSamples; subSampleRow++)
		{
			double xTemp = x + subSampleCol*deltaX + ((double) rand()/(RAND_MAX+1))*deltaX;
			double yTemp = y + subSampleRow*deltaY + ((double) rand()/(RAND_MAX+1))*deltaY;
			Vec3d tCol = trace(xTemp , yTemp);
			col += tCol;
		}
	}

	col = col/double(traceUI->m_nPixelSamples*traceUI->m_nPixelSamples);
	pixel[0] = (int)( 255.0 * col[0]);
	pixel[1] = (int)( 255.0 * col[1]);
	pixel[2] = (int)( 255.0 * col[2]);
	return col;
}

void RayTracer::setUseKdTree(bool kdTree)
{
    if (this->scene != nullptr)
    {
        this->scene->useKdTree = kdTree;
    }
}

void RayTracer::setBackFaceCulling(bool _backFace)
{
    if (this->scene != nullptr)
    {
        this->scene->backFaceCulling = _backFace;
    }
}

void RayTracer::setSmoothShading(bool _smoothShade)
{
	if (this->scene != nullptr)
    {
        this->scene->smoothShading = _smoothShade;
    }
}


// Do recursive ray tracing!  You'll want to insert a lot of code here
// (or places called from here) to handle reflection, refraction, etc etc.
Vec3d RayTracer::traceRay(ray& r, int depth)
{
	isect i;
	Vec3d colorC;
	//scene->useKdTree = traceUI->m_kdTree;
	// cout<<scene->useKdTree<<endl;
	if(scene->intersect(r, i)) {
		// YOUR CODE HERE

		// An intersection occurred!  We've got work to do.  For now,
		// this code gets the material for the surface that was intersected,
		// and asks that material to provide a color for the ray.  

		// This is a great place to insert code for recursive ray tracing.
		// Instead of just returning the result of shade(), add some
		// more steps: add in the contributions from reflected and refracted
		// rays.
		const Material& material = i.getMaterial();
		// Light Ray
		Vec3d intensity = material.shade(scene, r, i);
		if (depth == 0)
		{
			return intensity;
		}
		Vec3d Qpoint = r.at(i.t);
		Vec3d minusD = -1 * r.d;
		Vec3d cosVector = i.N * (minusD * i.N);
		Vec3d sinVector = cosVector + r.d;
		// Reflected Ray
		if (!material.kr(i).iszero())
		{
			Vec3d reflectedDirection = cosVector + sinVector;
			reflectedDirection.normalize();
			ray reflectedRay(Qpoint, reflectedDirection, ray::REFLECTION);
			intensity = intensity + prod(material.kr(i), traceRay(reflectedRay, depth - 1));
		}
		//Refracted Ray
		if (!material.kt(i).iszero())
		{
			double cosineAngle = acos(i.N * r.d) * 180/M_PI;
			double n_i, n_r;
			double criticalAngle = 360;
			int iDirection;
			// bool goingIn = true;
			// double cosThetaI = 0;
			if (cosineAngle > 90) // Coming into an object from air
			{
				n_i = 1;
				n_r = material.index(i);
				iDirection = 1;
				// cosThetaI = i.N * -1 * r.d;
			}
			else // Going out from object to air
			{
				n_i = material.index(i);
				n_r = 1;
				// goingIn = false;
				// cosThetaI = i.N * r.d;
				iDirection = -1;
			}
			double n = n_i/n_r;
			Vec3d sinT = (n_i/n_r) * sinVector;
			Vec3d cosT = (-1 * i.N) * sqrt(1 - sinT*sinT);
			if (cosineAngle < criticalAngle)
			{
				Vec3d refractedDirection = cosT + iDirection*sinT;
				refractedDirection.normalize();
				ray refractedRay(Qpoint, iDirection * refractedDirection, ray::REFRACTION);
				intensity = intensity + prod(material.kt(i), traceRay(refractedRay, depth -1));
			}
			// double sqrtTerm = 1 - (n*n)*(1 - cosThetaI*cosThetaI);
			// if (sqrtTerm > 0)
			// {
			// 	double cosThetaT = sqrt(sqrtTerm);
			// 	Vec3d refractedDirection = (n*cosThetaI - cosThetaT)*i.N - n*-1*r.d;
			// 	refractedDirection.normalize();
			// 	ray refractedRay(Qpoint, refractedDirection, ray::REFRACTION);
			// 	intensity = intensity + prod(material.kt(i), traceRay(refractedRay, depth -1));
			// }
		}
	  	colorC = intensity;
	} else {
		Vec3d intensity(0.0, 0.0, 0.0);
		if (traceUI->m_usingCubeMap && this->haveCubeMap())
		{
			intensity = this->getCubeMap()->getColor(r);
			colorC = intensity;
		}
		// No intersection.  This ray travels to infinity, so we color
		// it according to the background color, which in this (simple) case
		// is just black.
		colorC = intensity;
	}
	return colorC;
}

RayTracer::RayTracer()
	: scene(0), buffer(0), buffer_width(256), buffer_height(256), m_bBufferReady(false)
{}

RayTracer::~RayTracer()
{
	delete scene;
	delete [] buffer;
}

void RayTracer::getBuffer( unsigned char *&buf, int &w, int &h )
{
	buf = buffer;
	w = buffer_width;
	h = buffer_height;
}

void RayTracer::setBuffer ()
{
	// for (int i = 0; i < 20; i++)
	// {
	// 	for (int j = 0; j < 20; j++)
	// 	{
	// 		double x = double(i)/double(buffer_width);
	// 		double y = double(j)/double(buffer_height);

	// 		unsigned char *pixel = buffer + ( i + j * buffer_width ) * 3;

	// 		pixel[0] = (int)( 255.0 * 1);
	// 		pixel[1] = (int)( 255.0 * 1);
	// 		pixel[2] = (int)( 255.0 * 1);
	// 	}
	// }
}

double RayTracer::aspectRatio()
{
	return sceneLoaded() ? scene->getCamera().getAspectRatio() : 1;
}

bool RayTracer::loadScene( char* fn ) {
	ifstream ifs( fn );
	if( !ifs ) {
		string msg( "Error: couldn't read scene file " );
		msg.append( fn );
		traceUI->alert( msg );
		return false;
	}
	
	// Strip off filename, leaving only the path:
	string path( fn );
	if( path.find_last_of( "\\/" ) == string::npos ) path = ".";
	else path = path.substr(0, path.find_last_of( "\\/" ));

	// Call this with 'true' for debug output from the tokenizer
	Tokenizer tokenizer( ifs, false );
    Parser parser( tokenizer, path );
	try {
		delete scene;
		scene = 0;
		scene = parser.parseScene();
	} 
	catch( SyntaxErrorException& pe ) {
		traceUI->alert( pe.formattedMessage() );
		return false;
	}
	catch( ParserException& pe ) {
		string msg( "Parser: fatal exception " );
		msg.append( pe.message() );
		traceUI->alert( msg );
		return false;
	}
	catch( TextureMapException e ) {
		string msg( "Texture mapping exception: " );
		msg.append( e.message() );
		traceUI->alert( msg );
		return false;
	}
	if (traceUI->m_kdTree)
	{
		string temp( fn );
		size_t found = temp.find("turtle.ray");
  		if (found==std::string::npos)
  		{
  			scene->buildKdTree(traceUI->getKdMaxDepth(), traceUI->getKdLeafSize());
			scene->useKdTree = traceUI->m_kdTree;
  		}
	}
	this->setBackFaceCulling(traceUI->bfCulling());
	this->setSmoothShading(traceUI->smShadSw());
	if( !sceneLoaded() ) return false;

	return true;
}

void RayTracer::traceSetup(int w, int h)
{
	if (buffer_width != w || buffer_height != h)
	{
		buffer_width = w;
		buffer_height = h;
		bufferSize = buffer_width * buffer_height * 3;
		delete[] buffer;
		buffer = new unsigned char[bufferSize];
	}
	memset(buffer, 0, w*h*3);
	m_bBufferReady = true;
}

