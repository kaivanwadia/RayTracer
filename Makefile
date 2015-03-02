#
#  Makefile for fltk applications
#

INCLUDE = -I/lusr/opt/fltk1.3-1.3.0/include
LIBS = -L/lusr/opt/fltk1.3-1.3.0/lib -lfltk -lfltk_gl -lfltk_images -lGL -lGLU -ljpeg -lpng -lz

CFLAGS = -g -w -std=c++11 -O3 $(INCLUDE) $(LIBS) 
#CFLAGS = -O1 -std=c++11 $(INCLUDE) $(LIBS) 

CC = g++

.SUFFIXES: .o .cpp .cxx

.cpp.o: 
	$(CC) $(CFLAGS) -c -o $*.o $<

.cxx.o: 
	$(CC) $(CFLAGS) -c -o $*.o $<

ALL.O = src/main.o src/getopt.o src/RayTracer.o \
	src/ui/CommandLineUI.o src/ui/GraphicalUI.o src/ui/TraceGLWindow.o \
	src/ui/debuggingView.o src/ui/glObjects.o src/ui/debuggingWindow.o \
	src/ui/ModelerCamera.o src/ui/CubeMapChooser.o \
	src/fileio/bitmap.o src/fileio/buffer.o \
	src/fileio/pngimage.o \
	src/parser/Token.o src/parser/Tokenizer.o \
	src/parser/Parser.o src/parser/ParserException.o \
	src/scene/camera.o src/scene/light.o\
	src/scene/material.o src/scene/ray.o src/scene/scene.o \
	src/scene/cubeMap.o \
	src/SceneObjects/Box.o src/SceneObjects/Cone.o \
	src/SceneObjects/Cylinder.o src/SceneObjects/trimesh.o \
	src/SceneObjects/Sphere.o src/SceneObjects/Square.o

ray: $(ALL.O)
	$(CC) $(CFLAGS) -o $@ $(ALL.O) $(LIBS)

clean:
	rm -f $(ALL.O)

clean_all:
	rm -f $(ALL.O) ray