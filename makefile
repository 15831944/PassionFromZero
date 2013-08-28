## This is the makefile to generate obj/*.o of *.cc in src/

CC = g++
CFLAGS = -Wall -g
OBJDIR = obj
_OBJ = delta.o \
       binary search tree.o \
	   simple_HashTable.o \
	   single-linked list.o \
	   string_permute.o \
	   string_reverse.o \
	   string_rmDuplicate.o \
	   test_charTOint.o \
	   test1.o \
	   test2.o \
	   test3.o
OBJ= $(_OBJ:%=$(OBJDIR)/%)

#LIBDIR = ../lib
#DEP = GL/glut.h


.PHONY: clean

all: $(OBJDIR) $(OBJ) draw

draw: $(SUBDIR) $(OBJ)
	$(CC) $(OBJDIR)/* -o $@ -lglut $(CFLAGS)
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


