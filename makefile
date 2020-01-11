CC=gcc
CFLAGS=-std=c++17

# Dependencies
DEPSH = decoder.h Test.h utils.h
OBJ = main.o decoder.o Test.o
LIBS=-lopencv_calib3d -lopencv_imgproc -lopencv_core -lopencv_highgui -lopencv_imgcodecs -lstdc++ -lm

# Paths
IDIRS = -I. -I./libs -I/usr/include/opencv4
IDIR = ./
LDIR =../lib
ODIR=./obj

DEPS = $(patsubst %,$(IDIR)/%,$(DEPSH))
OBJS = $(patsubst %,$(ODIR)/%,$(OBJ))

$(ODIR)/%.o: %.cpp $(DEPS)
	$(CC) $(CFLAGS) $(IDIRS) -c -o $@ $< 

decoder: $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(IDIRS) $(LIBS)

.PHONY: clean

clean:
	rm -f $(ODIR)/*.o