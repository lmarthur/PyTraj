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
- [X] Implement ctypes wrapper for C functions - [X] Implement output file writer
- [X] Implement plotting functions
- [X] Write scripts for compiling, testing, and running
- [X] Write tests for flight initialization
- [X] Implement flight initialization function
- [X] Write analysis suite to fit distributions to accuracy data
- [X] Implement configuration file and parser
- [X] Write tests for monte carlo function
- [X] Implement monte carlo function in C
- [X] Write tests for imu module
- [X] Implement imu functions in C
- [X] Write tests for gnss module
- [X] Implement gnss functions in C
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

BUG REPORTS: None
