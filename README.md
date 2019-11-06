# GBRSim

## Overview
GBRSim is a point mass simulation of the Green Bath Racing electric
Urban Concept vehicle for the Shell Eco Marathon. This simulation was
written for the individual research aspect of my third year Group
Business and Design Project.

## Example Usage 
* An example simulation around the 2019 track is shown in
  *exampleSimulation.py*. 
* An example burn and coast optimisation is shown in
  *exampleOptimisation.py*.

## Content
This repository contains six subdirectories: Integration, Model, Track,
Strategy, Optimisation, Results.

### Integration
Contains numerical integrator implementations. These are all explicit
schemes. DP45 and RKF45 are adaptive time stepping schemes, the others
are fixed step. *Butcher.py* implements any explicit fixed step Runge
Kutta scheme from a Butcher Tableau.

### Model
Contains objects representing the physical systems being simulated. Each
system model inherits from Model. The vehicle models are children of
ContrainedParticle which models a point mass constrained to an arbitrary
curve of type Segment (found in Track directory). 

### Track
The race track is represented by a set of parametric curves implemented
as Segment objects. *ImportTrack.py* contains functions to fit cubic
bezier spline curves 3D points defined in a csv file. 

### Strategy
The throttle position is given by the different controller classes in
*Controller.py*. This subdirectory also contains wrappers for
optimisation and sensitivity analysis.

### Optimisation
Implements interface between simulation and SciPy optimiser schemes.
This is used to determine best driving strategy around the track as in
*exampleOptimisation.py*.

### Results
Contains MATLAB app to visualise results from simulation.
 
## Future Ideas
* A factory design pattern would make the instantiation of simulations
  neater to users.
* Currently information is propagated from vehicle model into powertrain
  into motor and back. This works well for this specific case but it
  could be more generalised into a set of independent systems with some
  managing object that propagates the information between systems at the
  same rate everywhere.