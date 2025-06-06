build:
	g++ main.cpp  -framework IOKit -framework Cocoa -framework OpenGL $(shell pkg-config --libs --cflags raylib) -o main
run:
	./main