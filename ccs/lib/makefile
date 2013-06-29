## This is the makefile to generate obj/*.o of *.cc in lib/

CC = g++
CFLAGS = -Wall
OBJDIR = obj
_OBJ = RecPoint3D.o

OBJ = $(patsubst %, $(OBJDIR)/%, $(_OBJ))
## the following is quivalent to the above
#OBJ = $(_OBJ:%=$(OBJDIR)/%)

.PHONY: clean

all: $(OBJDIR) $(OBJ)

$(OBJDIR):
	@mkdir -p $(OBJDIR)
#	@echo OBJ= $(OBJ)

$(OBJDIR)/%.o: %.cc %.h
	@$(CC) -c $< -o $@ $(CFLAGS)
	@echo "make $@ success"


clean:
	rm -rf $(OBJDIR)

