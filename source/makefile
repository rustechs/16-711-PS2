COMMON=-O2 -I../include -L../bin -std=c++11

default:
	g++ $(COMMON) cart-pole-sim.cpp -lmujoco140 -lGL -lglew ../bin/libglfw.so.3 -o ../bin/cart-pole

all: default
