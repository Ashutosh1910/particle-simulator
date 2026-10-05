UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Darwin)
    PLATFORM_LIBS = -framework IOKit -framework Cocoa -framework OpenGL
else
    PLATFORM_LIBS = -lGL -lm -lpthread -ldl -lrt -lX11
endif

build:
	g++ -O2 main.cpp $(shell pkg-config --libs --cflags raylib) $(PLATFORM_LIBS) -o main
run:
	./main
