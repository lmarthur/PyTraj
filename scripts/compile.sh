# This script compiles the program and test scripts, and runs the tests.
#!/bin/bash
# Compile the program
echo "Compiling the program..."

rm ./test/build/PyTraj_test

# Compile with CMake
cmake -S ./test -B test/build -Who-dev
make -C ./test/build

# Run the tests
echo "Running the tests..."
./test/build/PyTraj_test