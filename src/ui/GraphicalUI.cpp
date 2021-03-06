//
// GraphicalUI.cpp
//
// Handles FLTK integration and other user interface tasks
//
#include <stdio.h>
#include <time.h>
#include <string.h>
#include <stdarg.h>
#include <iostream>

#ifndef COMMAND_LINE_ONLY

#include <FL/fl_ask.H>
#include "debuggingView.h"

#include "GraphicalUI.h"
#include "../RayTracer.h"
#include <thread>

#define MAX_INTERVAL 500

#ifdef _WIN32
#define print sprintf_s
#else
#define print sprintf
#endif

bool GraphicalUI::stopTrace = false;
bool GraphicalUI::doneTrace = true;
GraphicalUI* GraphicalUI::pUI = NULL;
char* GraphicalUI::traceWindowLabel = "Raytraced Image";
bool TraceUI::m_debug = false;

//------------------------------------- Help Functions --------------------------------------------
GraphicalUI* GraphicalUI::whoami(Fl_Menu_* o)	// from menu item back to UI itself
{
	return ((GraphicalUI*)(o->parent()->user_data()));
}

//--------------------------------- Callback Functions --------------------------------------------
void GraphicalUI::cb_load_scene(Fl_Menu_* o, void* v) 
{
	pUI = whoami(o);

	static char* lastFile = 0;
	char* newfile = fl_file_chooser("Open Scene?", "*.ray", NULL );

	if (newfile != NULL) {
		char buf[256];

		if (pUI->raytracer->loadScene(newfile)) {
			print(buf, "Ray <%s>", newfile);
			stopTracing();	// terminate the previous rendering
		} else print(buf, "Ray <Not Loaded>");

		pUI->m_mainWindow->label(buf);
		pUI->m_debuggingWindow->m_debuggingView->setDirty();

		if( lastFile != 0 && strcmp(newfile, lastFile) != 0 )
			pUI->m_debuggingWindow->m_debuggingView->resetCamera();

		pUI->m_debuggingWindow->redraw();
	}
}

void GraphicalUI::cb_load_cubemap(Fl_Menu_* o, void* v) 
{
	pUI = whoami(o);
	if (pUI->m_cubeMapChooser != NULL)
	{
		pUI->m_cubeMapChooser->show();
	}
}

void GraphicalUI::cb_save_image(Fl_Menu_* o, void* v) 
{
	pUI = whoami(o);

	char* savefile = fl_file_chooser("Save Image?", "*.bmp", "save.bmp" );
	if (savefile != NULL) {
		pUI->m_traceGlWindow->saveImage(savefile);
	}
}

void GraphicalUI::cb_exit(Fl_Menu_* o, void* v)
{
	pUI = whoami(o);

	// terminate the rendering
	stopTracing();

	pUI->m_traceGlWindow->hide();
	pUI->m_mainWindow->hide();
	pUI->m_debuggingWindow->hide();
	TraceUI::m_debug = false;
}

void GraphicalUI::cb_exit2(Fl_Widget* o, void* v) 
{
	pUI = (GraphicalUI *)(o->user_data());

	// terminate the rendering
	stopTracing();

	pUI->m_traceGlWindow->hide();
	pUI->m_mainWindow->hide();
	pUI->m_debuggingWindow->hide();
	TraceUI::m_debug = false;
}

void GraphicalUI::cb_about(Fl_Menu_* o, void* v) 
{
	fl_message("RayTracer Project for CS384g.");
}

void GraphicalUI::cb_depthSlides(Fl_Widget* o, void* v)
{
	((GraphicalUI*)(o->user_data()))->m_nDepth=int( ((Fl_Slider *)o)->value() );
}

void GraphicalUI::cb_blockSizeSlides(Fl_Widget* o, void* v)
{
	((GraphicalUI*)(o->user_data()))->m_nBlockSize=int( ((Fl_Slider *)o)->value() );
}

void GraphicalUI::cb_thresholdSlides(Fl_Widget* o, void* v)
{
	((GraphicalUI*)(o->user_data()))->m_nThreshold=int( ((Fl_Slider *)o)->value() );
}

void GraphicalUI::cb_sizeSlides(Fl_Widget* o, void* v)
{
	pUI=(GraphicalUI*)(o->user_data());

	// terminate the rendering so we don't get crashes
	stopTracing();

	pUI->m_nSize=int(((Fl_Slider *)o)->value());
	int width = (int)(pUI->getSize());
	int height = (int)(width / pUI->raytracer->aspectRatio() + 0.5);
	pUI->m_traceGlWindow->resizeWindow(width, height);
}

void GraphicalUI::cb_refreshSlides(Fl_Widget* o, void* v)
{
	((GraphicalUI*)(o->user_data()))->refreshInterval=clock_t(((Fl_Slider *)o)->value()) ;
}

void GraphicalUI::cb_threadsSlides(Fl_Widget* o, void* v)
{
	((GraphicalUI*)(o->user_data()))->m_nThreads=int( ((Fl_Slider *)o)->value() );
}

void GraphicalUI::cb_aaCheckButton(Fl_Widget* o, void* v)
{
	pUI=(GraphicalUI*)(o->user_data());
	pUI->m_antiAlias = (((Fl_Check_Button*)o)->value() == 1);
	if (pUI->m_antiAlias)
	{
		pUI->m_aaSamplesSlider->activate();
		pUI->m_aaThreshSlider->activate();
	}
	else
	{
		pUI->m_aaSamplesSlider->deactivate();
		pUI->m_aaThreshSlider->deactivate();
	}
}

void GraphicalUI::cb_aaWhiteCheckButton(Fl_Widget* o, void* v)
{
	pUI=(GraphicalUI*)(o->user_data());
	pUI->m_antiAliasWhite = (((Fl_Check_Button*)o)->value() == 1);
}

void GraphicalUI::cb_aaSamplesSlides(Fl_Widget* o, void* v)
{
	((GraphicalUI*)(o->user_data()))->m_nPixelSamples=int( ((Fl_Slider *)o)->value() );
}

void GraphicalUI::cb_aaThresholdSlides(Fl_Widget* o, void* v)
{
	((GraphicalUI*)(o->user_data()))->m_nSupersampleThreshold=int( ((Fl_Slider *)o)->value() );
}

void GraphicalUI::cb_kdTreeCheckButton(Fl_Widget* o, void* v)
{
	pUI=(GraphicalUI*)(o->user_data());
	pUI->m_kdTree = (((Fl_Check_Button*)o)->value() == 1);
	pUI->getRayTracer()->setUseKdTree(pUI->m_kdTree);
	if (pUI->m_kdTree)
	{
		pUI->m_treeDepthSlider->activate();
		pUI->m_leafSizeSlider->activate();
	}
	else
	{
		pUI->m_treeDepthSlider->deactivate();
		pUI->m_leafSizeSlider->deactivate();
	}
}

void GraphicalUI::cb_maxDepthSlides(Fl_Widget* o, void* v)
{
	((GraphicalUI*)(o->user_data()))->m_nMaxDepth=int( ((Fl_Slider *)o)->value() );
}

void GraphicalUI::cb_leafSizeSlides(Fl_Widget* o, void* v)
{
	((GraphicalUI*)(o->user_data()))->m_nLeafSize=int( ((Fl_Slider *)o)->value() );
}

void GraphicalUI::cb_cubeMapCheckButton(Fl_Widget* o, void* v)
{
	pUI=(GraphicalUI*)(o->user_data());
	pUI->m_usingCubeMap = (((Fl_Check_Button*)o)->value() == 1);
	if (pUI->m_usingCubeMap)
	{
		pUI->m_filterSlider->activate();
	}
	else
	{
		pUI->m_filterSlider->deactivate();
	}
}

void GraphicalUI::cb_filterWidthSlides(Fl_Widget* o, void* v)
{
	((GraphicalUI*)(o->user_data()))->m_nFilterWidth=int( ((Fl_Slider *)o)->value() );
}

void GraphicalUI::cb_ssCheckButton(Fl_Widget* o, void* v)
{
	pUI=(GraphicalUI*)(o->user_data());
	pUI->m_smoothshade = (((Fl_Check_Button*)o)->value() == 1);
	pUI->raytracer->setSmoothShading(pUI->m_smoothshade);
}

void GraphicalUI::cb_shCheckButton(Fl_Widget* o, void* v)
{
	pUI=(GraphicalUI*)(o->user_data());
	pUI->m_shadows = (((Fl_Check_Button*)o)->value() == 1);
}

void GraphicalUI::cb_bfCheckButton(Fl_Widget* o, void* v)
{
	pUI=(GraphicalUI*)(o->user_data());
	pUI->m_bfCulling = (((Fl_Check_Button*)o)->value() == 1);
	pUI->getRayTracer()->setBackFaceCulling(pUI->m_bfCulling);
}

void GraphicalUI::cb_debuggingDisplayCheckButton(Fl_Widget* o, void* v)
{
	pUI=(GraphicalUI*)(o->user_data());
	pUI->m_displayDebuggingInfo = (((Fl_Check_Button*)o)->value() == 1);
	if (pUI->m_displayDebuggingInfo)
	  {
	    pUI->m_debuggingWindow->show();
	    pUI->m_debug = true;
	  }
	else
	  {
	    pUI->m_debuggingWindow->hide();
	    pUI->m_debug = false;
	  }
}

void GraphicalUI::renderThread(int threadNo, int width, int height, int noOfCols, RayTracer* rayTracer)
{
	int start = threadNo*noOfCols;
	int end = start + noOfCols;
	int count = 0;
	for (int y = 0; y < height; y++)
	{
		for (int x = start; x < end; x++)
		{
			if (stopTrace) break;
			rayTracer->tracePixel(x, y);
		}
		if (stopTrace) break;
	}
}

void GraphicalUI::cb_render(Fl_Widget* o, void* v) {
	char buffer[256];

	pUI = (GraphicalUI*)(o->user_data());
	doneTrace = stopTrace = false;
	if (pUI->raytracer->sceneLoaded())
	  {
		int width = pUI->getSize();
		int height = (int)(width / pUI->raytracer->aspectRatio() + 0.5);
		int origPixels = width * height;
		pUI->m_traceGlWindow->resizeWindow(width, height);
		pUI->m_traceGlWindow->show();
		pUI->raytracer->traceSetup(width, height);

		std::vector<std::thread> threads;
		int noOfCols = ceil((double)width/(double)pUI->m_nThreads);
		// cout<<"No Of Cols : "<<noOfCols<<endl;
		for (int i = 1; i < pUI->m_nThreads; i++)
		{
			threads.push_back(std::thread(renderThread, i, width, height, noOfCols, pUI->getRayTracer()));
		}
		// Save the window label
		const char *old_label = pUI->m_traceGlWindow->label();

		std::chrono::time_point<std::chrono::system_clock> start, end;
		start = std::chrono::system_clock::now();

		clock_t now, prev;
		clock_t tEnd, tStart = clock();
		now = prev = clock();
		clock_t intervalMS = pUI->refreshInterval * 100;
		for (int y = 0; y < height; y++)
		  {
		    for (int x = 0; x < noOfCols; x++)
		      {
			if (stopTrace) break;
			// check for input and refresh view every so often while tracing
			now = clock();
			if ((now - prev)/CLOCKS_PER_SEC * 1000 >= intervalMS)
			  {
			    prev = now;
			    // sprintf(buffer, "(%d%%) %s", (int)((double)y / (double)height * 100.0), old_label);
			    // pUI->m_traceGlWindow->label(buffer);
			    pUI->m_traceGlWindow->refresh();
			    Fl::check();
			    if (Fl::damage()) { Fl::flush(); }
			  }
			// look for input and refresh window
			pUI->raytracer->tracePixel(x, y);
			pUI->m_debuggingWindow->m_debuggingView->setDirty();
		      }
		    if (stopTrace) break;
		  }
		doneTrace = true;
		stopTrace = false;
		// Restore the window label
		for (int i = 0; i < pUI->m_nThreads - 1; i++)
		{
			threads[i].join();
		}
		end = std::chrono::system_clock::now();
		std::chrono::duration<double> elapsed_seconds = end-start;
		sprintf(buffer, "%f MS To RENDER ", elapsed_seconds.count() * 1000);
		pUI->m_traceGlWindow->label(buffer);
		pUI->m_traceGlWindow->refresh();
		if(pUI->m_antiAlias)
		{
			doAntiAliasing(pUI);
		}
	  }
}

void GraphicalUI::antiAliasRenderThread(int threadNo, int width, int height, int noOfCols, RayTracer* rayTracer)
{
	int start = threadNo*noOfCols;
	int end = start + noOfCols;
	for (int y = 0; y < height; y++)
	{
		for (int x = start; x < end; x++)
		{
			if (stopTrace) break;
			if(pUI->raytracer->filteredBuf[(y*width + x)] == 255)
			{
				// cout<<"Here"<<endl;
				pUI->raytracer->tracePixelAntiAlias(x, y);
			}
			if (stopTrace) break;
		}
		if (stopTrace) break;
	}
}

void GraphicalUI::doAntiAliasing(GraphicalUI* pUI)
{
	clock_t now, prev;
	now = prev = clock();
	clock_t intervalMS = pUI->refreshInterval * 100;
	doneTrace = stopTrace = false;
	unsigned char* buf;

	char buffer[256];
	const char *old_label = pUI->m_traceGlWindow->label();
	sprintf(buffer, "ANTI ALIASING START %s ", old_label);
	pUI->m_traceGlWindow->label(buffer);
	pUI->m_traceGlWindow->refresh();


	int width, height;

	pUI->raytracer->getBuffer(buf, width, height);
	pUI->raytracer->filteredBuf = new unsigned char[width * height];
	applyFilter(buf, width, height, pUI->raytracer->filteredBuf, pUI->m_nSupersampleThreshold);

	std::vector<std::thread> aaThreads;
	int noOfCols = ceil((double)width/(double)pUI->m_nThreads);
	for (int i = 1; i < pUI->m_nThreads; i++)
	{
		aaThreads.push_back(std::thread(antiAliasRenderThread, i, width, height, noOfCols, pUI->getRayTracer()));
	}
	// Do edge detection on filteredBuf
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < noOfCols; x++)
		{
			if (stopTrace) break;
			if(pUI->raytracer->filteredBuf[(y*width + x)] == 255)
			{
				pUI->raytracer->tracePixelAntiAlias(x, y);
				now = clock();
				if ((now - prev)/CLOCKS_PER_SEC * 1000 >= intervalMS)
			  	{
				    prev = now;
				    pUI->m_traceGlWindow->refresh();
				    Fl::check();
				    if (Fl::damage()) { Fl::flush(); }
			  	}
			}

		}
		if (stopTrace) break;
	}
	for (int i = 0; i < pUI->m_nThreads - 1; i++)
	{
		aaThreads[i].join();
	}
	doneTrace = true;
	stopTrace = false;
	sprintf(buffer, "ANTI ALIASING DONE %s ", old_label);
	pUI->m_traceGlWindow->label(buffer);
	pUI->m_traceGlWindow->refresh();
}

void GraphicalUI::applyFilter( const unsigned char* sourceBuffer,
		int srcBufferWidth, int srcBufferHeight,
		unsigned char* destBuffer, int cutOff)
{
	unsigned char*	grayImage;
	grayImage = new unsigned char[srcBufferWidth*srcBufferHeight];
	for (int imageRow = 0; imageRow < srcBufferHeight; imageRow++)
	{
		for (int imageCol = 0; imageCol < srcBufferWidth; imageCol++)
		{
			double gray = 0.299*sourceBuffer[3 * (imageRow*srcBufferWidth + imageCol) + 0] + 0.587*sourceBuffer[3 * (imageRow*srcBufferWidth + imageCol) + 1] + 0.114*sourceBuffer[3 * (imageRow*srcBufferWidth + imageCol) + 2];
			grayImage[imageRow*srcBufferWidth + imageCol] = gray;
		}
	}

	int knlWidth, knlHeight;
	knlWidth = knlHeight = 3;
	int offset = 128;
	int filterKernel[9] = { 0, 1, 0, 1, -4, 1, 0, 1, 0 };
	for (int pixelRow = 0; pixelRow < srcBufferHeight; pixelRow++)
	{
		for (int pixelColumn = 0; pixelColumn < srcBufferWidth; pixelColumn++)
		{
			double sum = 0;
			for (int filterRow = 0; filterRow < knlWidth; filterRow++)
			{
				for (int filterColumn = 0; filterColumn < knlHeight; filterColumn++)
				{
					int tempPixelRow = pixelRow - knlHeight/2 + filterRow;
					int tempPixelColumn = pixelColumn - knlWidth/2 + filterColumn;
					if (tempPixelRow < 0 || tempPixelColumn < 0 || tempPixelRow>srcBufferHeight || tempPixelColumn>srcBufferWidth)
					{
						continue;
					}
					sum = sum + filterKernel[knlWidth * filterRow + filterColumn] * grayImage[(tempPixelRow*srcBufferWidth + tempPixelColumn)];
				}
			}
			// sum = sum / 1 + offset;
			if (sum > cutOff)
			{
				sum = 255;
			}
			else
			{
				sum = 0;
			}
			destBuffer[(pixelRow*srcBufferWidth + pixelColumn)] = sum;
		}
	}
}

void GraphicalUI::cb_stop(Fl_Widget* o, void* v)
{
	pUI = (GraphicalUI*)(o->user_data());
	stopTracing();
}

int GraphicalUI::run()
{
	Fl::visual(FL_DOUBLE|FL_INDEX);

	m_mainWindow->show();

	return Fl::run();
}

void GraphicalUI::alert( const string& msg )
{
	fl_alert( "%s", msg.c_str() );
}

void GraphicalUI::setRayTracer(RayTracer *tracer)
{
	// printf("In setRayTracer()\n");
	TraceUI::setRayTracer(tracer);
	this->raytracer = tracer;
	// std::cout<<this->raytracer<<std::endl;
	m_traceGlWindow->setRayTracer(tracer);
	m_debuggingWindow->m_debuggingView->setRayTracer(tracer);
}

// menu definition
Fl_Menu_Item GraphicalUI::menuitems[] = {
	{ "&File", 0, 0, 0, FL_SUBMENU },
	{ "&Load Scene...",	FL_ALT + 'l', (Fl_Callback *)GraphicalUI::cb_load_scene },
	{ "&Load Cube Map...",	FL_ALT + 'c', (Fl_Callback *)GraphicalUI::cb_load_cubemap },
	{ "&Save Image...", FL_ALT + 's', (Fl_Callback *)GraphicalUI::cb_save_image },
	{ "&Exit", FL_ALT + 'e', (Fl_Callback *)GraphicalUI::cb_exit },
	{ 0 },

	{ "&Help",		0, 0, 0, FL_SUBMENU },
	{ "&About",	FL_ALT + 'a', (Fl_Callback *)GraphicalUI::cb_about },
	{ 0 },

	{ 0 }
};

void GraphicalUI::stopTracing()
{
	stopTrace = true;
}

GraphicalUI::GraphicalUI() : refreshInterval(10) {
	// init.
	m_mainWindow = new Fl_Window(100, 40, 450, 459, "Ray <Not Loaded>");
	m_mainWindow->user_data((void*)(this));	// record self to be used by static callback functions
	// install menu bar
	m_menubar = new Fl_Menu_Bar(0, 0, 440, 25);
	m_menubar->menu(menuitems);

	// set up "render" button
	m_renderButton = new Fl_Button(360, 37, 70, 25, "&Render");
	m_renderButton->user_data((void*)(this));
	m_renderButton->callback(cb_render);

	// set up "stop" button
	m_stopButton = new Fl_Button(360, 65, 70, 25, "&Stop");
	m_stopButton->user_data((void*)(this));
	m_stopButton->callback(cb_stop);

	// install depth slider
	m_depthSlider = new Fl_Value_Slider(10, 40, 180, 20, "Recursion Depth");
	m_depthSlider->user_data((void*)(this));	// record self to be used by static callback functions
	m_depthSlider->type(FL_HOR_NICE_SLIDER);
	m_depthSlider->labelfont(FL_COURIER);
	m_depthSlider->labelsize(12);
	m_depthSlider->minimum(0);
	m_depthSlider->maximum(10);
	m_depthSlider->step(1);
	m_depthSlider->value(m_nDepth);
	m_depthSlider->align(FL_ALIGN_RIGHT);
	m_depthSlider->callback(cb_depthSlides);

	// install Block Size slider
	m_blockSlider = new Fl_Value_Slider(10, 65, 180, 20, "Block Size");
	m_blockSlider->user_data((void*)(this));	// record self to be used by static callback functions
	m_blockSlider->type(FL_HOR_NICE_SLIDER);
	m_blockSlider->labelfont(FL_COURIER);
	m_blockSlider->labelsize(12);
	m_blockSlider->minimum(2);
	m_blockSlider->maximum(64);
	m_blockSlider->step(1);
	m_blockSlider->value(m_nBlockSize);
	m_blockSlider->align(FL_ALIGN_RIGHT);
	m_blockSlider->callback(cb_blockSizeSlides);

	// install Threshold slider
	m_thresholdSlider = new Fl_Value_Slider(10, 90, 180, 20, "Threshold (x 0.001)");
	m_thresholdSlider->user_data((void*)(this));	// record self to be used by static callback functions
	m_thresholdSlider->type(FL_HOR_NICE_SLIDER);
	m_thresholdSlider->labelfont(FL_COURIER);
	m_thresholdSlider->labelsize(12);
	m_thresholdSlider->minimum(0);
	m_thresholdSlider->maximum(1000);
	m_thresholdSlider->step(1);
	m_thresholdSlider->value(m_nThreshold);
	m_thresholdSlider->align(FL_ALIGN_RIGHT);
	m_thresholdSlider->callback(cb_thresholdSlides);

	// install size slider
	m_sizeSlider = new Fl_Value_Slider(10, 115, 180, 20, "Screen Size");
	m_sizeSlider->user_data((void*)(this));	// record self to be used by static callback functions
	m_sizeSlider->type(FL_HOR_NICE_SLIDER);
	m_sizeSlider->labelfont(FL_COURIER);
	m_sizeSlider->labelsize(12);
	m_sizeSlider->minimum(64);
	m_sizeSlider->maximum(1024);
	m_sizeSlider->step(2);
	m_sizeSlider->value(m_nSize);
	m_sizeSlider->align(FL_ALIGN_RIGHT);
	m_sizeSlider->callback(cb_sizeSlides);

	// install refresh interval slider
	m_refreshSlider = new Fl_Value_Slider(10, 140, 180, 20, "Screen Refresh Interval (0.1 sec)");
	m_refreshSlider->user_data((void*)(this));	// record self to be used by static callback functions
	m_refreshSlider->type(FL_HOR_NICE_SLIDER);
	m_refreshSlider->labelfont(FL_COURIER);
	m_refreshSlider->labelsize(12);
	m_refreshSlider->minimum(1);
	m_refreshSlider->maximum(300);
	m_refreshSlider->step(1);
	m_refreshSlider->value(refreshInterval);
	m_refreshSlider->align(FL_ALIGN_RIGHT);
	m_refreshSlider->callback(cb_refreshSlides);

	// install threads slider
	m_threadSlider = new Fl_Value_Slider(10, 165, 180, 20, "Threads");
	m_threadSlider->user_data((void*)(this));	// record self to be used by static callback functions
	m_threadSlider->type(FL_HOR_NICE_SLIDER);
	m_threadSlider->labelfont(FL_COURIER);
	m_threadSlider->labelsize(12);
	m_threadSlider->minimum(1);
	m_threadSlider->maximum(32);
	m_threadSlider->step(1);
	m_threadSlider->value(m_nThreads);
	m_threadSlider->align(FL_ALIGN_RIGHT);
	m_threadSlider->callback(cb_threadsSlides);

	// set up antialias checkbox
	m_aaCheckButton = new Fl_Check_Button(10, 220, 100, 20, "Antialias");
	m_aaCheckButton->user_data((void*)(this));
	m_aaCheckButton->callback(cb_aaCheckButton);
	m_aaCheckButton->value(m_antiAlias);

	// set up antialias checkbox
	m_aaWhiteCheckButton = new Fl_Check_Button(10, 250, 100, 20, "AA Edges");
	m_aaWhiteCheckButton->user_data((void*)(this));
	m_aaWhiteCheckButton->callback(cb_aaWhiteCheckButton);
	m_aaWhiteCheckButton->value(m_antiAliasWhite);

	// install Pixel Samples slider
	m_aaSamplesSlider = new Fl_Value_Slider(110, 210, 180, 20, "Pixel Samples Width");
	m_aaSamplesSlider->user_data((void*)(this));	// record self to be used by static callback functions
	m_aaSamplesSlider->type(FL_HOR_NICE_SLIDER);
	m_aaSamplesSlider->labelfont(FL_COURIER);
	m_aaSamplesSlider->labelsize(12);
	m_aaSamplesSlider->minimum(1);
	m_aaSamplesSlider->maximum(4);
	m_aaSamplesSlider->step(1);
	m_aaSamplesSlider->value(m_nPixelSamples);
	m_aaSamplesSlider->align(FL_ALIGN_RIGHT);
	m_aaSamplesSlider->callback(cb_aaSamplesSlides);
	m_aaSamplesSlider->deactivate();

	// install Super sample threshold slider
	m_aaThreshSlider = new Fl_Value_Slider(110, 240, 180, 20, "Supersample Threshold");
	m_aaThreshSlider->user_data((void*)(this));	// record self to be used by static callback functions
	m_aaThreshSlider->type(FL_HOR_NICE_SLIDER);
	m_aaThreshSlider->labelfont(FL_COURIER);
	m_aaThreshSlider->labelsize(12);
	m_aaThreshSlider->minimum(1);
	m_aaThreshSlider->maximum(255);
	m_aaThreshSlider->step(1);
	m_aaThreshSlider->value(m_nSupersampleThreshold);
	m_aaThreshSlider->align(FL_ALIGN_RIGHT);
	m_aaThreshSlider->callback(cb_aaThresholdSlides);
	m_aaThreshSlider->deactivate();

	// set up k-d Tree checkbox
	m_kdCheckButton = new Fl_Check_Button(10, 290, 100, 20, "K-d Tree");
	m_kdCheckButton->user_data((void*)(this));
	m_kdCheckButton->callback(cb_kdTreeCheckButton);
	m_kdCheckButton->value(m_kdTree);

	// install Max Depth slider
	m_treeDepthSlider = new Fl_Value_Slider(110, 280, 180, 20, "Max Depth");
	m_treeDepthSlider->user_data((void*)(this));	// record self to be used by static callback functions
	m_treeDepthSlider->type(FL_HOR_NICE_SLIDER);
	m_treeDepthSlider->labelfont(FL_COURIER);
	m_treeDepthSlider->labelsize(12);
	m_treeDepthSlider->minimum(1);
	m_treeDepthSlider->maximum(30);
	m_treeDepthSlider->step(1);
	m_treeDepthSlider->value(m_nMaxDepth);
	m_treeDepthSlider->align(FL_ALIGN_RIGHT);
	m_treeDepthSlider->callback(cb_maxDepthSlides);

	// install Target Leaf size slider
	m_leafSizeSlider = new Fl_Value_Slider(110, 310, 180, 20, "Target Leaf Size");
	m_leafSizeSlider->user_data((void*)(this));	// record self to be used by static callback functions
	m_leafSizeSlider->type(FL_HOR_NICE_SLIDER);
	m_leafSizeSlider->labelfont(FL_COURIER);
	m_leafSizeSlider->labelsize(12);
	m_leafSizeSlider->minimum(1);
	m_leafSizeSlider->maximum(100);
	m_leafSizeSlider->step(1);
	m_leafSizeSlider->value(m_nLeafSize);
	m_leafSizeSlider->align(FL_ALIGN_RIGHT);
	m_leafSizeSlider->callback(cb_leafSizeSlides);

	// set up Cube Map checkbox
	m_cubeMapCheckButton = new Fl_Check_Button(10, 350, 100, 20, "CubeMap");
	m_cubeMapCheckButton->user_data((void*)(this));
	m_cubeMapCheckButton->callback(cb_cubeMapCheckButton);
	m_cubeMapCheckButton->value(m_usingCubeMap);

	// install filter width slider
	m_filterSlider = new Fl_Value_Slider(110, 350, 180, 20, "Filter Width");
	m_filterSlider->user_data((void*)(this));	// record self to be used by static callback functions
	m_filterSlider->type(FL_HOR_NICE_SLIDER);
	m_filterSlider->labelfont(FL_COURIER);
	m_filterSlider->labelsize(12);
	m_filterSlider->minimum(1);
	m_filterSlider->maximum(32);
	m_filterSlider->step(1);
	m_filterSlider->value(m_nFilterWidth);
	m_filterSlider->align(FL_ALIGN_RIGHT);
	m_filterSlider->callback(cb_filterWidthSlides);
	m_filterSlider->deactivate();

	// set up smoothshade checkbox
	m_ssCheckButton = new Fl_Check_Button(10, 400, 140, 20, "Smoothshade");
	m_ssCheckButton->user_data((void*)(this));
	m_ssCheckButton->callback(cb_ssCheckButton);
	m_ssCheckButton->value(m_smoothshade);

	// set up shadows checkbox
	// m_shCheckButton = new Fl_Check_Button(160, 400, 100, 20, "Shadows");
	// m_shCheckButton->user_data((void*)(this));
	// m_shCheckButton->callback(cb_shCheckButton);
	// m_shCheckButton->value(m_shadows);
	// m_shCheckButton->deactivate();
	// m_shCheckButton->hide();

	// set up Backface Cull checkbox
	m_bfCheckButton = new Fl_Check_Button(160, 400, 140, 20, "Backface Cull");
	m_bfCheckButton->user_data((void*)(this));
	m_bfCheckButton->callback(cb_bfCheckButton);
	m_bfCheckButton->value(m_bfCulling);

	// set up debugging display checkbox
	m_debuggingDisplayCheckButton = new Fl_Check_Button(10, 429, 140, 20, "Debugging display");
	m_debuggingDisplayCheckButton->user_data((void*)(this));
	m_debuggingDisplayCheckButton->callback(cb_debuggingDisplayCheckButton);
	m_debuggingDisplayCheckButton->value(m_displayDebuggingInfo);

	m_mainWindow->callback(cb_exit2);
	m_mainWindow->when(FL_HIDE);
	m_mainWindow->end();

	// image view
	m_traceGlWindow = new TraceGLWindow(100, 150, m_nSize, m_nSize, traceWindowLabel);
	m_traceGlWindow->end();
	m_traceGlWindow->resizable(m_traceGlWindow);

	// debugging view
	m_debuggingWindow = new DebuggingWindow();
	m_cubeMapChooser = new CubeMapChooser();
	m_cubeMapChooser->setCaller(this);
}

#endif