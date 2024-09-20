# This script compiles the program and test scripts, and runs the tests.
#!/bin/bash
# Compile the program
echo "Compiling the program..."

rm ./test/build/PyTraj_test
rm ./build/libPyTraj.so

# Compile with CMake
cmake -S ./test -B test/build -Wno-dev
make -C ./test/build

# Run the tests
echo "Running the tests..."
./test/build/PyTraj_test

# Compile the shared library
echo "Compiling the shared library..."
cc -shared -fPIC -o ./build/libPyTraj.so ./src/main.c

echo "Done."