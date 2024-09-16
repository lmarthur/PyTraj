L.M. Arthur, Cambridge, MA, 2024

This code is a work in progress, and is not yet ready for use. It is being developed for research purposes at the MIT Laboratory for Nuclear Security and Policy. 

# Overview
This code is a Python package for simulating the flight of a ballistic missile. It is designed to be modular, with separate modules for geodesy, atmosphere, vehicle, gravity, stepper, integrator, flight, monte carlo, guidance, control, and Kalman filtering. The code is designed to be fast and efficient, and is written in a combination of Python and C.

TODO: 
- [ ] Write geodesy tests with tau
- [ ] Implement geodesy functions in C
- [ ] Set up CMake 
- [ ] Write tests for atmosphere module
- [ ] Implement atmosphere functions in C
- [ ] Write tests for vehicle module
- [ ] Implement minuteman model in C
- [ ] Write tests for gravity module
- [ ] Implement gravity functions in C
- [ ] Write tests for stepper
- [ ] Implement stepper in C
- [ ] Determine whether to implement integration in C or Python
- [ ] Write tests for integrator
- [ ] Implement integrator
- [ ] Implement CFFI
- [ ] Write tests for flight module
- [ ] Implement flight function in Python
- [ ] Write tests for monte carlo function
- [ ] Implement monte carlo function in Python
- [ ] Implement configuration file and parser
- [ ] Implement output file writer
- [ ] Implement plotting functions
- [ ] Write scripts for compiling, testing, and running
- [ ] Write analysis suite to fit distributions to accuracy data
- [ ] Write tests for guidance module
- [ ] Implement guidance functions in C
- [ ] Write tests for control module
- [ ] Implement control functions in C
- [ ] Write tests for Kalman filter
- [ ] Implement Kalman filter in C
- [ ] Set up documentation
- [ ] Write function for parameter scanning
- [ ] Write function to generate sensitivity plots
