g++ -o test fpstestLD.cpp utils/sources/posest.cpp `pkg-config opencv --cflags --libs` -fopenmp -std=gnu++14
./test
rm test
