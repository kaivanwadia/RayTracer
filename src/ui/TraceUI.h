//
// rayUI.h
//
// The header file for the UI part
//

#ifndef __rayUI_h__
#define __rayUI_h__

// who the hell cares if my identifiers are longer than 255 characters:
#pragma warning(disable : 4786)

#include <string>

using std::string;

class RayTracer;

class TraceUI {
public:
	TraceUI() : m_nDepth(0), m_nSize(512), m_displayDebuggingInfo(false),
					m_shadows(true), m_smoothshade(true), raytracer(0),
                    m_nFilterWidth(1), m_nBlockSize(4), m_nThreshold(0),
                    m_nThreads(8), m_bfCulling(true), m_antiAlias(false),
                    m_kdTree(true), m_usingCubeMap(false), m_gotCubeMap(false),
                    m_nMaxDepth(15), m_nLeafSize(10), m_nPixelSamples(3),
                    m_nSupersampleThreshold(100)
                    {}
	virtual int	run() = 0;

	// Send an alert to the user in some manner
	virtual void alert(const string& msg) = 0;

	// setters
	virtual void setRayTracer( RayTracer* r ) { raytracer = r; }
	void setCubeMap(bool b) { m_gotCubeMap = b; }
	void useCubeMap(bool b) { m_usingCubeMap = b; }

	// accessors:
	int	getDepth() const { return m_nDepth; }
	int getBlockSize() const { return m_nBlockSize; }
	int getThreshold() const { return m_nThreshold; }
	int	getSize() const { return m_nSize; }
	int getThreads() const { return m_nThreads; }
	int getKdMaxDepth() const { return m_nMaxDepth; }
	int getKdLeafSize() const { return m_nLeafSize; }
	
	int	getFilterWidth() const { return m_nFilterWidth; }	

	bool	shadowSw() const { return m_shadows; }
	bool	smShadSw() const { return m_smoothshade; }
	bool	antiAliasing() const { return m_antiAlias; }
	bool	kdTree() const { return m_kdTree; }
	bool	bfCulling() const { return m_bfCulling; }
	bool	displayDebugInfo() const { return m_displayDebuggingInfo; }
	bool	usingCubeMap() const { return m_usingCubeMap; }
	bool	gotCubeMap() const { return m_gotCubeMap; }

	static bool m_debug;

protected:
	RayTracer*	raytracer;

	int	m_nSize;	// Size of the traced image
	int	m_nDepth;	// Max depth of recursion
	int m_nBlockSize; // Size of the block
	int m_nThreshold; // Normal Threshold
	int m_nThreads; // Number of threads
	bool m_antiAlias; // Using anti aliasing
	int m_nPixelSamples; // Pixel Samples for anti aliasing
	int m_nSupersampleThreshold; // Supersample threshold for antialiasing
	bool m_kdTree; // Using k-d Trees
	int m_nMaxDepth; // The max depth of the K-d Tree
	int m_nLeafSize; // Size of the leaves in K-d Tree

	// Determines whether or not to show debugging information
	// for individual rays.  Disabled by default for efficiency
	// reasons.
	bool m_displayDebuggingInfo;
	bool m_shadows;  // compute shadows?
	bool m_smoothshade;  // turn on/off smoothshading?
	bool m_bfCulling;	// Using backface culling
	bool m_usingCubeMap;  // render with cubemap
	bool m_gotCubeMap;  // cubemap defined
	int m_nFilterWidth;  // width of cubemap filter
};

#endif
