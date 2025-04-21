CXX = g++
CXXFLAGS = `pkg-config --cflags opencv4` -I"$(WEBOTS_HOME)/include/controller/cpp"
LDFLAGS = `pkg-config --libs opencv4` -L"$(WEBOTS_HOME)/lib/controller" -lCppController

SRC = obstacle.c
OUT = obstacle

all:
	$(CXX) $(SRC) -o $(OUT) $(CXXFLAGS) $(LDFLAGS)

clean:
	rm -f $(OUT)
