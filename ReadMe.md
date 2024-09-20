L.M. Arthur, Cambridge, MA, 2024

This code is a work in progress, and is not yet ready for use. It is being developed for research purposes at the MIT Laboratory for Nuclear Security and Policy. 

# Overview
This code is a Python package for simulating the flight of a ballistic missile. It is designed to be modular, with separate modules for geodesy, atmosphere, vehicle, gravity, stepper, integrator, flight, monte carlo, guidance, control, and Kalman filtering. The code is designed to be fast and efficient, and is written in a combination of Python and C.

TODO: 
- [X] Set up CMake 
- [X] Write tests for atmosphere module
- [X] Implement exp atmosphere functions in C
- [X] Write tests for vehicle module
- [X] Implement minuteman model in C
- [X] Write tests for newtonian gravity module
- [X] Implement newtonian gravity functions in C
- [X] Write tests for physics module
- [X] Implement physics functions in C
- [X] Write tests for stepper
- [X] Implement stepper in C
- [X] Determine whether to implement integration in C or Python
- [X] Write tests for flight module
- [X] Implement flight function
- [X] Implement ctypes wrapper for C functions
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
- [ ] Write tests for gravity perturbations
- [ ] Implement gravity perturbations in C
- [ ] Write C wrapper for EarthGRAM with shared object
- [ ] Set up documentation
- [ ] Write function for parameter scanning
- [ ] Write function to generate sensitivity plots

BUG REPORTS:
- [ ] Vertically launched missiles fly less high than missiles launched at an angle
