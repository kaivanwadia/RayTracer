//
// GraphicalUI.h
//
// The header file for the graphical UI
//

#ifndef __GraphicalUI_h__
#define __GraphicalUI_h__

#include <FL/Fl.H>
#include <FL/Fl_Window.H>
#include <FL/Fl_Menu_Bar.H>
#include <FL/Fl_Value_Slider.H>
#include <FL/Fl_Check_Button.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_File_Chooser.H>

#include "TraceUI.h"
#include "TraceGLWindow.h"
#include "debuggingWindow.h"
#include "CubeMapChooser.h"

class ModelerView;

class GraphicalUI : public TraceUI {
public:
	GraphicalUI();

	int run();

	void alert( const string& msg );

	// The FLTK widgets
	Fl_Window*			m_mainWindow;
	Fl_Menu_Bar*		m_menubar;

	Fl_Slider*			m_depthSlider;
	Fl_Slider*			m_blockSlider;
	Fl_Slider*			m_thresholdSlider;
	Fl_Slider*			m_sizeSlider;
	Fl_Slider*			m_refreshSlider;
	Fl_Slider*			m_threadSlider;
	Fl_Slider*			m_aaSamplesSlider;
	Fl_Slider*			m_aaThreshSlider;
	Fl_Slider*			m_treeDepthSlider;
	Fl_Slider*			m_leafSizeSlider;
	Fl_Slider*			m_filterSlider;

	Fl_Check_Button*	m_aaCheckButton;
	Fl_Check_Button*	m_aaWhiteCheckButton;
	Fl_Check_Button*	m_kdCheckButton;
	Fl_Check_Button*	m_cubeMapCheckButton;
	Fl_Check_Button*	m_ssCheckButton;
	Fl_Check_Button*	m_shCheckButton;
	Fl_Check_Button*	m_bfCheckButton;
	Fl_Check_Button*	m_debuggingDisplayCheckButton;

	Fl_Button*			m_renderButton;
	Fl_Button*			m_stopButton;

	CubeMapChooser*     m_cubeMapChooser;

	TraceGLWindow*		m_traceGlWindow;

	DebuggingWindow*	m_debuggingWindow;

	// member functions
	void setRayTracer(RayTracer *tracer);
	RayTracer* getRayTracer() { return raytracer; }
	clock_t getRefreshInterval() {return refreshInterval; }

	static void stopTracing();

	// static vars
	static char *traceWindowLabel;
	
private:

	clock_t refreshInterval;

	// static class members
	static Fl_Menu_Item menuitems[];

	static GraphicalUI* whoami(Fl_Menu_* o);

	static void cb_load_scene(Fl_Menu_* o, void* v);
	static void cb_load_cubemap(Fl_Menu_* o, void* v);
	static void cb_save_image(Fl_Menu_* o, void* v);
	static void cb_exit(Fl_Menu_* o, void* v);
	static void cb_about(Fl_Menu_* o, void* v);

	static void cb_exit2(Fl_Widget* o, void* v);
	static void cb_exit3(Fl_Widget* o, void* v);

	static void cb_sizeSlides(Fl_Widget* o, void* v);
	static void cb_blockSizeSlides(Fl_Widget* o, void* v);
	static void cb_thresholdSlides(Fl_Widget* o, void* v);
	static void cb_depthSlides(Fl_Widget* o, void* v);
	static void cb_refreshSlides(Fl_Widget* o, void* v);
	static void cb_threadsSlides(Fl_Widget* o, void* v);

	static void cb_render(Fl_Widget* o, void* v);
	static void renderThread(int threadNo, int width, int height, int noOfCols, RayTracer* rayTracer);
	static void cb_stop(Fl_Widget* o, void* v);
	
	static void cb_debuggingDisplayCheckButton(Fl_Widget* o, void* v);
	static void cb_ssCheckButton(Fl_Widget* o, void* v);
	static void cb_shCheckButton(Fl_Widget* o, void* v);
	static void cb_bfCheckButton(Fl_Widget* o, void* v);

	static void cb_aaCheckButton(Fl_Widget* o, void* v);
	static void cb_aaWhiteCheckButton(Fl_Widget* o, void* v);
	static void cb_aaSamplesSlides(Fl_Widget* o, void* v);
	static void cb_aaThresholdSlides(Fl_Widget* o, void* v);
	static void cb_kdTreeCheckButton(Fl_Widget* o, void* v);
	static void cb_maxDepthSlides(Fl_Widget* o, void* v);
	static void cb_leafSizeSlides(Fl_Widget* o, void* v);
	static void cb_cubeMapCheckButton(Fl_Widget* o, void* v);
	static void cb_filterWidthSlides(Fl_Widget* o, void* v);

	static void doAntiAliasing(GraphicalUI* pUI);
	static void antiAliasRenderThread(int threadNo, int width, int height, int noOfCols, RayTracer* rayTracer);
	static void applyFilter(const unsigned char* sourceBuffer,
		int srcBufferWidth, int srcBufferHeight,
		unsigned char* destBuffer, int cutOff);

	static bool stopTrace;
	static bool doneTrace;
	static GraphicalUI* pUI;
};

#endif
