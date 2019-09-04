CXXFLAGS := -lpthread -lm -std=c++11
all:logger show
logger:logger.o
	g++ -o $@ $^ $(CXXFLAGS)
show:show.o
	g++ -o $@ $^ $(CXXFLAGS)
logger.o:logger.cpp
	g++ -c -o $@ $^ $(CXXFLAGS)
show.o:show.cpp
	g++ -c -o $@ $^ $(CXXFLAGS)	
clean:
	rm logger.o logger show.o show
