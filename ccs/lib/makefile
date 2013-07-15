## This is the makefile to generate obj/*.o of *.cc in lib/

CC = g++
CFLAGS = -Wall -g
OBJDIR = obj
_OBJ = RecRadians.o          \
	   RecPoint2D.o          \
	   RecPoint3D.o          \
	   RecVector2D.o         \
	   RecVector3D.o         \
	   RecPose2D.o           \
	   RecPose3D.o           \
	   RecTransform2D.o      \
	   RecTransform3D.o      \
	   RecQuaternion.o       \
	   RecLineSegment2D.o    \
	   RecLineSegment3D.o    \
	   RecAxisAlignedBox2D.o \
	   RecAxisAlignedBox3D.o \
	   RecPolygon2D.o

OBJ  = $(patsubst %, $(OBJDIR)/%, $(_OBJ))
## the following is quivalent to the above
#OBJ = $(_OBJ:%=$(OBJDIR)/%)

.PHONY: clean

all: $(OBJDIR) $(OBJ)

$(OBJDIR):
	@mkdir -p $(OBJDIR)
#	@echo OBJ= $(OBJ)

$(OBJDIR)/%.o: %.cc RecGeometry.h
	@$(CC) -c $< -o $@ $(CFLAGS)
	@echo "Build $@ success"


clean:
	rm -rf $(OBJDIR)

