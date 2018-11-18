# Select the robot (MINITAUR, or MINITAUR_E)
ROBOT=MINITAUR_E

# Flags
SDK_LOCATION=../..
CFLAGS=-DNDEBUG -std=gnu++11 -I$(SDK_LOCATION)/inc -I$(SDK_LOCATION)/thirdparty/nanopb -I/usr/include/eigen3 -L$(SDK_LOCATION)/lib -DROBOT_$(ROBOT) -DARCH_gazebo
LDLIBS=-L/opt/ros/kinetic/lib -lroscpp -lrosconsole -lroscpp_serialization -lrostime -L$(SDK_LOCATION)/lib -lgazebo -lroscpp -lrosconsole -lroscpp_serialization -lrostime -Wl,-rpath,/opt/ros/kinetic/lib
ARCH=
all:
	@$(ARCH)g++ $(BFLAGS) $(CFLAGS) -c -o main.o main.cpp
	@$(ARCH)g++ -std=gnu++11 -O1 main.o  -o FirstHop_$(ROBOT) $(LDLIBS)

clean:
	rm -f main.elf FirstHop_$(ROBOT) main.o
