CC=gcc
CFLAGS=-std=c++17 -O3

# Dependencies
DEPSH = decoder.h Test.h utils.h core_app.h quantizer.h chunk_utils.h
OBJ = main.o decoder.o Test.o core_app.o quantizer.o chunk_utils.o
LIBS=-lopencv_calib3d -lopencv_imgproc -lopencv_core -lopencv_highgui -lopencv_imgcodecs -lopencv_videoio -lstdc++ -lm

# Paths
IDIRS = -I. -I./libs -I/usr/include/opencv4
IDIR = ./include
LDIR =../lib
ODIR=./obj
ISRC=./src

DEPS = $(patsubst %,$(IDIR)/%,$(DEPSH))
OBJS = $(patsubst %,$(ODIR)/%,$(OBJ))

$(ODIR)/%.o: $(ISRC)/%.cpp $(DEPS)
	$(CC) $(CFLAGS) $(IDIRS) -c -o $@ $< 

decoder: $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(IDIRS) $(LIBS)

.PHONY: clean

clean:
	rm -f $(ODIR)/*.o