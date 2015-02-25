#
#  Makefile for fltk applications
#

LIBS = -lfltk -lfltk_gl -lfltk_images -lGL -lGLU -ljpeg -lpng -lz

CFLAGS = -g -w -std=c++11 -Wno-write-strings -Wno-deprecated-declarations

CC = g++

ALL = $(addprefix lib/src/, main getopt RayTracer \
	ui/CommandLineUI ui/GraphicalUI ui/TraceGLWindow \
	ui/debuggingView ui/glObjects ui/debuggingWindow \
	ui/ModelerCamera ui/CubeMapChooser \
	fileio/bitmap fileio/buffer \
	fileio/pngimage \
	parser/Token parser/Tokenizer \
	parser/Parser parser/ParserException \
	scene/camera scene/light\
	scene/material scene/ray scene/scene \
	scene/cubeMap \
	SceneObjects/Box SceneObjects/Cone \
	SceneObjects/Cylinder SceneObjects/trimesh \
	SceneObjects/Sphere SceneObjects/Square \
	)
ALL.O = $(addsuffix .o, $(ALL))
ALL.MK = $(addsuffix .mk, $(ALL))


EXECS=$(addprefix bin/,ray)
default: $(EXECS)
	@true # do not tell be if it is already built kthxbye

lib/%.mk: %.cpp
	@mkdir -p $(@D)
	@$(CC) -MM -o $@ $<
	@sed -i 's/$(notdir $*).o:/$(subst /,\/,$@) lib\/$(subst /,\/,$*).o:/' $@
lib/%.mk: %.cxx
	@mkdir -p $(@D)
	@$(CC) -MM -o $@ $<
	@sed -i 's/$(notdir $*).o:/$(subst /,\/,$@) lib\/$(subst /,\/,$*).o:/' $@

-include $(ALL.MK)
MKDIRS=lib bin
$(MKDIRS):
	@mkdir -p $@

lib/%.o: %.cpp
	@mkdir -p $(@D)
	$(CC) $(CFLAGS) -c -o $@ $<
lib/%.o: %.cxx
	@mkdir -p $(@D)
	$(CC) $(CFLAGS) -c -o $@ $<

bin/ray: $(ALL.O) | bin
	$(CC) $(CFLAGS) -o $@ $(ALL.O) $(LIBS)

clean:
	rm -rf $(MKDIRS)
