# Aircraft Dynamics and Attitude Simulation

This MATLAB implementation provides comprehensive tools for simulating fixed-wing aircraft dynamics, originally developed by Prof. Luis Beningno.

## Features

- Complete 6-DOF aircraft mathematical model
- Trim condition calculations for:
  - Rectilinear flight
  - Turning flight
  - Pull-up maneuvers
  - Roll maneuvers
- Linearized aircraft model generation
- Dynamic mode analysis (phugoid, short period, Dutch roll, etc.)
- Visualization tools for simulation results

## Getting Started

1. Clone this repository
2. Open MATLAB and navigate to the repository folder
3. Run one of the test scripts:
   - `simulateNavionAircraft.m` for full simulation
   - `testTrimConditionsRect.m` for trim calculations
   - `testLinearizeAircraftModel.m` for linearization

## Example Usage

```matlab
% Define aircraft parameters
aircraft = defineNavionAircraft();

% Set initial conditions
x0 = [100; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0]; % [u,v,w,p,q,r,phi,theta,psi,X,Y,Z]

% Run simulation
[t,X,Y] = simulateAircraft(100, x0, zeros(4,1), [0;0;0], zeros(3,1), aircraft);

% Plot results
plotAircraftSimulationResults(t,X,Y,aircraft);
