CXXFLAGS := -lpthread -lm -std=c++11
all:logger_bin logger show
logger_bin:logger_bin.o
	g++ -o $@ $^ $(CXXFLAGS)
logger:logger.o
	g++ -o $@ $^ $(CXXFLAGS)
show:show.o
	g++ -o $@ $^ $(CXXFLAGS)
logger_bin.o:logger_bin.cpp
	g++ -c -o $@ $^ $(CXXFLAGS)
logger.o:logger.cpp
	g++ -c -o $@ $^ $(CXXFLAGS)
show.o:show.cpp
	g++ -c -o $@ $^ $(CXXFLAGS)	
clean:
	rm logger.o logger show.o show
