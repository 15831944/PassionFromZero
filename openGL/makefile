## This is the makefile to generate obj/*.o of *.cc in src/

CC = g++
CFLAGS = -Wall -g -lglut
OBJDIR = obj
_OBJ = createWindow.o
#	   standalone_seg.o
OBJ= $(_OBJ:%=$(OBJDIR)/%)

#LIBDIR = ../lib
#DEP = GL/glut.h


.PHONY: clean draw

all: draw

draw: $(OBJDIR) $(OBJ)
	$(CC) $(OBJDIR)/* -o $@ $(CFLAGS)
	@echo "Make Success!"

$(OBJDIR):
	@mkdir -p $(OBJDIR)
#	@echo OBJ= $(OBJ)

$(OBJDIR)/%.o: %.cc
	@$(CC) -c $< -o $@ $(CFLAGS)
	@echo "Build $@ success"

#####
clean:
	rm -rf $(OBJDIR) draw


## if use make's implicit rules of compiling %.o, then  %.c & command( $(CC) -c $< -o $@ )  can be omitted
#standalone_seg.o: standalone_seg.h STLneed.h RecPoint3D.h
#main.o: standalone_seg.h STLneed.h RecPoint3D.h


