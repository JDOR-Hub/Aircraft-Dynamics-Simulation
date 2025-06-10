# Aircraft Dynamics and Attitude Simulation

MATLAB implementation of fixed-wing aircraft dynamics and attitude simulation, originally developed by Prof. Luis Beningno. This repository includes core functions for aircraft modeling, simulation, and visualization.

## Available Functions

### Core Simulation
- `faircraft.m` - Main aircraft mathematical model (6-DOF equations)
- `simulateAircraft.m` - Runs aircraft simulation
- `testSimulateAircraft.m` - Test script for simulation

### Aerodynamics & Propulsion
- `aerodynamics.m` - Calculates aerodynamic forces and moments
- `propulsion.m` - Calculates propulsion system forces and moments

### Reference Frames & Transformations
- `Cbwmatrix.m` - Rotation matrix from wind to body frame
- `DCM.m` - Direction Cosine Matrix calculation
- `H.m` - Transformation matrix for attitude rates

### Environment
- `atmosphere.m` - ISA atmospheric model (density, pressure, temperature)

### Visualization
- `plotAircraftSimulationResults.m` - Plots simulation results

### Aircraft Definition
- `Navion_aircraft.m` - Contains Navion aircraft parameters

### Utility
- `isOctave.m` - Detects if running in Octave vs MATLAB

## Getting Started

1. Clone this repository:
   ```bash
   git clone https://github.com/yourusername/aircraft-dynamics-simulation.git
2. Open MATLAB/Octave and navigate to the repository folder
3. Run the test script:
   ```bash
   testSimulateAircraft

 ### Basic Usage Example
```bash
% Load aircraft parameters
aircraft = Navion_aircraft();

% Set initial conditions [u,v,w,p,q,r,phi,theta,psi,X,Y,Z]
x0 = [100; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0]; 

% Control inputs: [deltat; ih; deltae; deltaa; deltar]
delta = zeros(5,1);

% Wind velocity (Vwe) and CG offset (deltaCGb)
Vwe = [0; 0; 0];
deltaCGb = [0; 0; 0];

% Run 100-second simulation
[t,X,Y] = simulateAircraft(100, x0, delta, Vwe, deltaCGb, aircraft);

% Plot results
plotAircraftSimulationResults(t,X,Y,aircraft);
