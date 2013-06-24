# This is the makefile to run the standalone ccs algorithm

CC = g++
CFLAGS = -Wall -g
OBJ = standalone_main.o standalone_seg.o STLneed.o
DEP = standalone_seg.h STLneed.h


all: ccs

standalone_seg.o: standalone_seg.cc standalone_seg.h
	$(CC) -c $< -o $@

STLneed.o: STLneed.cc STLneed.h
	$(CC) -c $< -o $@

standalone_main.o: standalone_main.cc
	$(CC) -c $< -o $@

## the following: $(DEP) doesn't give us header file dependency as expected!
#%.o: %.cc $(DEP)
#	$(CC) -c $< -o $@

ccs: $(OBJ)
	$(CC) $^ -o $@ $(CFLAGS)
	@echo "Make Success!"


#####
clean:
	rm -f *.o ccs


.PHONY: clean
