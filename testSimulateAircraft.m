clear all;
clc;
close all;

% testSimulateAircraft.m

% recall aircraft data structure
Navion_aircraft
% relative CG position respect to nominal CG
deltaCGb = [0;0;0];

% define simulation time
tfinal = 120;

% wind expressed in earth frame
Vwe = [0;0;0];

% define initial conditions and controls vector
% initial altitude
h0 = aircraft.h;
% initial position
pe0 = [0;0;-h0];
% initial airspeed, angle of attack and angle of sideslip
V0 = aircraft.V - 1*1852/3600;
alpha0 = 0.5*pi/180;
beta0 = 0*pi/180;
% initial Euler angles
phi0 = 0*pi/180;
theta0 = 0.5*pi/180;
psi0 = 0*pi/180;
% initial angular velocity
omegab0 = [0;0;0];

% calculate initial velocity respect to air expressed in body frame
Vrelb0 = [V0*cos(alpha0)*cos(beta0); V0*sin(beta0); V0*sin(alpha0)*cos(beta0)];
Phi0 = [phi0; theta0; psi0];
Cbe = DCM(Phi0);
Vb0 = Vrelb0 + Cbe*Vwe;

% assemble initial state vector
x0 = [pe0;Phi0;Vb0;omegab0];

% aircraft controls
deltat = 0.3369;
deltaf = 0*pi/180;
ih = 0*pi/180;
deltae = -0.25*pi/180;
deltaa = 0*pi/180;
deltar = 0*pi/180;

% assemble aircraft controls vector
delta = [deltat;deltaf;ih;deltae;deltaa;deltar];

% simulate aircraft under these conditions
[t,X,Y] = simulateAircraft(tfinal,x0,delta,Vwe,deltaCGb,aircraft);

% plot simulation results
plotAircraftSimulationResults(t,X,Y,aircraft);

