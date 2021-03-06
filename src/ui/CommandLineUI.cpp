
#include <iostream>
#include <time.h>
#include <thread>
#include <stdarg.h>

#include <assert.h>

#include "CommandLineUI.h"
#include "../fileio/bitmap.h"

#include "../RayTracer.h"

using namespace std;

// The command line UI simply parses out all the arguments off
// the command line and stores them locally.
CommandLineUI::CommandLineUI( int argc, char* const* argv )
	: TraceUI()
{
	int i;

	progName=argv[0];

	while( (i = getopt( argc, argv, "tr:w:h:" )) != EOF )
	{
		switch( i )
		{
			case 'r':
				m_nDepth = atoi( optarg );
				break;

			case 'w':
				m_nSize = atoi( optarg );
				break;
			default:
			// Oops; unknown argument
			std::cerr << "Invalid argument: '" << i << "'." << std::endl;
			usage();
			exit(1);
		}
	}

	if( optind >= argc-1 )
	{
		std::cerr << "no input and/or output name." << std::endl;
		exit(1);
	}

	rayName = argv[optind];
	imgName = argv[optind+1];
}

void CommandLineUI::renderThread(int threadNo, int width, int height, int noOfCols, RayTracer* rayTracer)
{
	int start = threadNo*noOfCols;
	int end = start + noOfCols;
	int count = 0;
	for (int y = 0; y < height; y++)
	{
		for (int x = start; x < end; x++)
		{
			rayTracer->tracePixel(x, y);
		}
	}
}

int CommandLineUI::run()
{
	assert( raytracer != 0 );
	raytracer->loadScene( rayName );

	if( raytracer->sceneLoaded() )
	{
		int width = m_nSize;
		int height = (int)(width / raytracer->aspectRatio() + 0.5);

		raytracer->traceSetup( width, height );

		clock_t start, end;
		start = clock();

		std::vector<std::thread> threads;
		int noOfCols = ceil((double)width/(double)this->m_nThreads);
		// cout<<"No Of Cols : "<<noOfCols<<endl;
		for (int i = 1; i < this->m_nThreads; i++)
		{
			threads.push_back(std::thread(renderThread, i, width, height, noOfCols, raytracer));
		}
		for( int y = 0; y < height; ++y )
		{
			for( int x = 0; x < noOfCols; ++x )
			{
				raytracer->tracePixel(x,y);
			}
		}
		for (int i = 0; i < this->m_nThreads - 1; i++)
		{
			threads[i].join();
		}
		end=clock();

		// save image
		unsigned char* buf;

		raytracer->getBuffer(buf, width, height);

		if (buf)
			writeBMP(imgName, width, height, buf);

		double t=(double)(end-start)/CLOCKS_PER_SEC;
//		int totalRays = TraceUI::resetCount();
//		std::cout << "total time = " << t << " seconds, rays traced = " << totalRays << std::endl;
		std::cout << "total time = " << t << std::endl;
		return 0;
	}
	else
	{
		std::cerr << "Unable to load ray file '" << rayName << "'" << std::endl;
		return( 1 );
	}
}

void CommandLineUI::alert( const string& msg )
{
	std::cerr << msg << std::endl;
}

void CommandLineUI::usage()
{
	std::cerr << "usage: " << progName << " [options] [input.ray output.bmp]" << std::endl;
	std::cerr << "  -r <#>      set recursion level (default " << m_nDepth << ")" << std::endl; 
	std::cerr << "  -w <#>      set output image width (default " << m_nSize << ")" << std::endl;
}
