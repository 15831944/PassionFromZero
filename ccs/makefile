## This is the makefile to generate executable ccs

CC = g++
CFLAGS = -Wall -g
SUBDIR = lib src
OBJDIR_LIB = lib/obj
OBJDIR_SRC = src/obj


.PHONY: $(SUBDIR) clean

all: ccs

ccs: $(SUBDIR)
	$(CC) $(OBJDIR_LIB)/* $(OBJDIR_SRC)/* -o $@ $(CFLAGS)
	@echo "Make Success!"

$(SUBDIR):
	@$(MAKE) -C $@

#####
clean:
	rm -f ccs *.txt

cleanall: MAKE = make clean
cleanall: $(SUBDIR) clean



