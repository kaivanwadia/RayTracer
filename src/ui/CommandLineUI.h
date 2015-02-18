//
// CommandLineUI.h
//
// The header file for the command line UI
//

#ifndef __CommandLineUI_h__
#define __CommandLineUI_h__

#include "TraceUI.h"

// ***********************************************************
// from getopt.cpp
//#ifdef _WIN32
extern int getopt(int argc, char **argv, char *optstring);
//#else
extern int getopt(int argc, char* const* argv, char *optstring);
//#endif
extern char* optarg;
extern int optind, opterr, optopt;
// ***********************************************************

class CommandLineUI : public TraceUI {

public:
	CommandLineUI( int argc, char* const* argv );
	int		run();

	void		alert( const string& msg );

private:
	void		usage();

	char*	rayName;
	char*	imgName;
	char*	progName;
};

#endif
