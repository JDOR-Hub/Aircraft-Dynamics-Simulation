% propulsion.m
% function to calculate propulsion system forces and moments
% usage
%   [Ftb,Mtb] = propulsion(deltat,deltaCGb,aircraft)
% where
%   deltat : deltat : Propulsion system control (0<=deltat<=1)
%   deltaCGb : relative CG position respect to nominal CG expressed in body frame (m)
%   aircraft : aircraft data structure 
%   Ftb : net propulsion system force expressed in body frame (N)
%   Mtb : net propulsion system moment expressed in body frame (Nm)

function [Ftb,Mtb] = propulsion(deltat,deltaCGb,aircraft)
    Ftb = [1;0;0]*aircraft.Tmax*deltat;
    Mtb = -cross(deltaCGb,Ftb);
end