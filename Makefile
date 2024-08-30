CFLAGS = -Wl,--hash-style=gnu -Wall -g -pthread -lstdc++
#CC = g++

SRCS = realtime_serial.cpp
OBJS = $(SRCS:.cpp=.o)
LDFLAGS = -lmodbus 
OUTPUT = rttest


all: $(OUTPUT)

$(OUTPUT): $(OBJS)
	$(CXX) $(CFLAGS) -o $(OUTPUT) $(OBJS) $(LDFLAGS)

%.o: %.cpp $(HEADERS)
	$(CXX) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OUTPUT) $(OBJS)
