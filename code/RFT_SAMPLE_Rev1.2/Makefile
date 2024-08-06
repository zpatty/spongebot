.SUFFIXES : .cpp .c .o
CXX = g++
CC = g++
INC = -I./ -I../include -I../../include 
LDFLAGS =
LIBS =  -L./ -L../lib -L../../Lib -lpthread
CXXFLAGS = -g $(INC) 
CFLAGS=-g $(INC) 
RTFLAGS=-lrt

OBJS = program.o 
OBJS += RFT_UART_SAMPLE.o
OBJS += ../include/RFT_IF_PACKET_Rev1.2.o

TARGET = program
all : $(TARGET)
$(TARGET) : $(OBJS)
	$(CXX) $(RTFLAGS) -o $@ $(OBJS) $(LIBS)
clean :
	rm -rf $(OBJS)
new :
	$(MAKE) clean
	$(MAKE)

