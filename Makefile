# Particle Simulator
#
#   make            build the simulator (./psim)
#   make run        build and run it
#   make test       build and run the headless unit tests
#   make bench      build and run the broad-phase benchmark
#   make GPU=1      enable the OpenGL 4.3 compute path (Linux/Windows; needs raylib
#                   built with OPENGL_VERSION=4.3, e.g. `make raylib-gl43` below)
#
# raylib is found with pkg-config. Point RAYLIB_PREFIX at a custom install to
# use that instead (it must contain include/ and lib/).

CXX      ?= g++
CXXFLAGS ?= -O2
CXXFLAGS += -std=c++17 -Wall -Wextra -MMD -MP
# separate object directories per configuration, so `make GPU=1` after `make` rebuilds
BUILD    := build$(if $(filter 1,$(GPU)),-gpu)

UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Darwin)
    PLATFORM_LIBS = -framework IOKit -framework Cocoa -framework OpenGL
else
    PLATFORM_LIBS = -lGL -lm -lpthread -ldl -lrt -lX11
endif

ifdef RAYLIB_PREFIX
    RAYLIB_CFLAGS = -isystem $(RAYLIB_PREFIX)/include
    RAYLIB_LIBS   = -L$(RAYLIB_PREFIX)/lib -lraylib
else
    RAYLIB_CFLAGS = $(patsubst -I%,-isystem %,$(shell pkg-config --cflags raylib))
    RAYLIB_LIBS   = $(shell pkg-config --libs raylib)
endif

ifeq ($(GPU),1)
    CXXFLAGS += -DPSIM_GPU
endif

# Relink psim whenever the configuration changes (e.g. `make` then `make GPU=1`)
CONFIG := GPU=$(GPU) RAYLIB_PREFIX=$(RAYLIB_PREFIX)
$(shell mkdir -p build; echo '$(CONFIG)' | cmp -s - build/config || echo '$(CONFIG)' > build/config)

CORE_SRC  := $(wildcard src/core/*.cpp src/broadphase/*.cpp src/physics/*.cpp)
APP_SRC   := $(wildcard src/app/*.cpp)
TEST_SRC  := $(wildcard tests/*.cpp)
BENCH_SRC := bench/bench.cpp

CORE_OBJ  := $(CORE_SRC:%.cpp=$(BUILD)/%.o)
APP_OBJ   := $(APP_SRC:%.cpp=$(BUILD)/%.o)
TEST_OBJ  := $(TEST_SRC:%.cpp=$(BUILD)/%.o)
BENCH_OBJ := $(BENCH_SRC:%.cpp=$(BUILD)/%.o)

.PHONY: all build run test bench clean raylib-gl43

all: psim
build: psim

psim: $(CORE_OBJ) $(APP_OBJ) build/config
	$(CXX) $(CORE_OBJ) $(APP_OBJ) -o $@ $(RAYLIB_LIBS) $(PLATFORM_LIBS)

psim_tests: $(CORE_OBJ) $(TEST_OBJ)
	$(CXX) $^ -o $@ -lpthread

psim_bench: $(CORE_OBJ) $(BENCH_OBJ)
	$(CXX) $^ -o $@ -lpthread

# only the app needs raylib headers
$(BUILD)/src/app/%.o: src/app/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(RAYLIB_CFLAGS) -isystem third_party -c $< -o $@

$(BUILD)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

run: psim
	./psim

test: psim_tests
	./psim_tests

bench: psim_bench
	./psim_bench

clean:
	rm -rf build build-gpu psim psim_tests psim_bench

# Builds raylib 5.5 with OpenGL 4.3 into ./third_party/raylib-gl43 (for GPU=1):
#   make raylib-gl43 && make GPU=1 RAYLIB_PREFIX=third_party/raylib-gl43
raylib-gl43:
	rm -rf $(BUILD)/raylib-src
	git clone --depth 1 --branch 5.5 https://github.com/raysan5/raylib.git $(BUILD)/raylib-src
	cmake -S $(BUILD)/raylib-src -B $(BUILD)/raylib-src/build -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=OFF \
	      -DOPENGL_VERSION=4.3 -DCMAKE_INSTALL_PREFIX=$(abspath third_party/raylib-gl43)
	cmake --build $(BUILD)/raylib-src/build -j
	cmake --install $(BUILD)/raylib-src/build

-include $(CORE_OBJ:.o=.d) $(APP_OBJ:.o=.d) $(TEST_OBJ:.o=.d) $(BENCH_OBJ:.o=.d)
