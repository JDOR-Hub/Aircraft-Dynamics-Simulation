% simulateAircraft.m
% function to simulate a fixed wing aircraft modeled by faircraft function
% usage [t,X,Y] = simulateAircraft(tfinal,x0,delta,Vwe,deltaCGb,aircraft)
% where
%   tfinal : simulation time (s)
%   x0 =   [pe0;Phi0;Vb0;omegab0] : initial state vector
%   delta : [deltat;deltaf;ih;deltae;deltaa;deltar]: aircraft controls vector
%   Vwe : wind velocity expressed in earth frame (m/s)
%   deltaCGb : relative CG position respect to nominal CG expressed in body frame (m)
%   aircraft : aircraft data structure 
% t : times vector (s)
% X: matrix with simulation results. Row ith represents the state vector (x') 
%    at time t(i)
% Y : matrix where row ith represents the values of [V, alpha, beta] at
%     time t(i)

function [t,X,Y] = simulateAircraft(tfinal,x0,delta,Vwe,deltaCGb,aircraft)
    % create and initialize global variable Vbdot
    global Vbdot
    Vbdot = [0;0;0];
    % set options for numerical method to solve differential equations
    options = odeset('AbsTol',1e-3,'RelTol',1e-3,'MaxStep',...
        0.01,'InitialStep',0.01);
    
    % solve equations of motion for 0<=t<=tfinal 
    [t,X] = ode45(@(t,x) faircraft(t,x,delta,Vwe,deltaCGb, aircraft), [0 tfinal], x0, options);

    % calculate Y
    n = length(t);
    Y = zeros(n,3);
    for i=1:n
        Phi = X(i,4:6)';
        Vb = X(i,7:9)';
        Cbe = DCM(Phi);
        Vrelb = Vb - Cbe*Vwe;
        V = norm(Vrelb);
        alpha = atan(Vrelb(3)/Vrelb(1));
        beta = asin(Vrelb(2)/V);
        Y(i,:) = [V,alpha,beta];
    end
end