## This is the makefile to generate obj/*.o of *.cc in src/

CC = g++
CFLAGS = -Wall -g
OBJDIR = obj
_OBJ = main.o \
	   standalone_seg.o
OBJ= $(_OBJ:%=$(OBJDIR)/%)

LIBDIR = ../lib
DEP = standalone_seg.h STLneed.h           \
	  $(LIBDIR)/RecGeometry.h              \
	  $(LIBDIR)/header-only/ScrollingMap.h \
	  $(LIBDIR)/header-only/Map.h          \
	  $(LIBDIR)/header-only/TimeStamp.h


.PHONY: clean

all: $(OBJDIR) $(OBJ)

$(OBJDIR):
	@mkdir -p $(OBJDIR)
#	@echo OBJ= $(OBJ)

$(OBJDIR)/%.o: %.cc $(DEP)
	@$(CC) -c $< -o $@ $(CFLAGS)
	@echo "Build $@ success"


clean:
	rm -rf $(OBJDIR)


## if use make's implicit rules of compiling %.o, then  %.c & command( $(CC) -c $< -o $@ )  can be omitted
#standalone_seg.o: standalone_seg.h STLneed.h RecPoint3D.h
#main.o: standalone_seg.h STLneed.h RecPoint3D.h


