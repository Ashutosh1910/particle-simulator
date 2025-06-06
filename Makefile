build:
	eval g++ main.cpp  -framework IOKit -framework Cocoa -framework OpenGL $(pkg-config --libs --cflags raylib) -o main
run:
	./main