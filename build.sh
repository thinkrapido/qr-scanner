g++ -g3 -std=c++0x `pkg-config --cflags opencv zbar` -o `basename image.cpp .cpp` -I. Circle.cpp image.cpp `pkg-config --libs opencv zbar`
