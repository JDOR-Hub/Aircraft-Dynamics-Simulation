% aerodynamics.m
% function to calculate aerodynamic forces and moments for a fixed wing aircraft
% usage
%   [Fab,Mab] = aerodynamics(V,alpha,beta,alphadot,omegab,deltaaero,qbar,M,deltaCGb,aircraft)
% where
% V : airspeed (m/s)
% alpha : angle of attack (rad)
% beta : angle of sideslip (rad)
% alphadot : derivative of angle of attack (rad/s)
% omegab  : [p;q;r]aircraft angular velocity respect to earth expressed 
%           in body frame (rad/s)
% deltaaero : [deltaf;ih;deltae;deltaa;deltar] : aircraft aerodynamic control vector
%   with deltat : Propulsion system control (0<=deltat<=1)       
%       deltaf : flap (rad)
%       ih :  Horizontal tail incidence (rad)
%       deltae : elevador (rad) 
%       deltaa : Aileron (rad)
%       deltar : rudder (rad)
% qbar : dynamic pressure (Pa)
% M : mach number
% deltaCGb : relative CG position respect to nominal CG expressed in body frame (m)
% aircraft : aircraft data structure
% Fab : net aerodynamic force expressed in body frame (N)
% Mab : net aerodynamic moment expressed in body frame (Nm)

function [Fab,Mab] = aerodynamics(V,alpha,beta,alphadot,omegab,deltaaero,qbar,M,deltaCGb,aircraft)

    % extract components of omegab and deltaaero
    p = omegab(1,1);
    q = omegab(2,1);
    r = omegab(3,1);
    deltaf = deltaaero(1,1);
    ih = deltaaero(2,1);
    deltae = deltaaero(3,1);
    deltaa = deltaaero(4,1);
    deltar = deltaaero(5,1);
    
    % calculate dimensionless aerodynamic force coefficients
    CL = aircraft.CL0 + aircraft.CLalpha*alpha + aircraft.CLdeltaf*deltaf+...
        aircraft.CLih*ih + aircraft.CLdeltae*deltae +...
        aircraft.cbar/(2*V)*(aircraft.CLalphadot*alphadot + aircraft.CLq*q)+...
        aircraft.CLM*(M-aircraft.M0);
    
    CD = aircraft.CD0 + (CL-aircraft.CLmindrag)^2/(pi*aircraft.A*aircraft.e)+...
        aircraft.CDM*(M-aircraft.M0);
    
    CC = aircraft.CCbeta*beta + aircraft.CCdeltaa*deltaa + aircraft.CCdeltar*deltar+...
        aircraft.b/(2*V)*(aircraft.CCp*p + aircraft.CCr*r);
    
    % denormalize aerodynamic forces
    L = qbar*aircraft.S*CL;
    D = qbar*aircraft.S*CD;
    C = qbar*aircraft.S*CC;
    
    % assemble net aerodynamic force expressed in wind frame
    Faw = [-D;-C;-L]; 
    
    % calculate net aerodynamic force expressed in body frame
    Cbw = Cbwmatrix(alpha,beta);
    Fab = Cbw*Faw;
    
    % calculate dimensionless aerodynamic moment coefficients
    Cl = aircraft.Clbeta*beta + aircraft.Cldeltaa*deltaa + aircraft.Cldeltar*deltar+...
        aircraft.b/(2*V)*(aircraft.Clp*p + aircraft.Clr*r);
    
    Cm = aircraft.Cm0 + aircraft.Cmalpha*alpha + aircraft.Cmdeltaf*deltaf+...
        aircraft.Cmih*ih + aircraft.Cmdeltae*deltae +...
        aircraft.cbar/(2*V)*(aircraft.Cmalphadot*alphadot + aircraft.Cmq*q)+...
        aircraft.CmM*(M-aircraft.M0);
    
    Cn = aircraft.Cnbeta*beta + aircraft.Cndeltaa*deltaa + aircraft.Cndeltar*deltar+...
        aircraft.b/(2*V)*(aircraft.Cnp*p + aircraft.Cnr*r);
    
    % denormalize aerodynamic moments
    l = qbar*aircraft.S*aircraft.b*Cl;
    m = qbar*aircraft.S*aircraft.cbar*Cm;
    n = qbar*aircraft.S*aircraft.b*Cn;
    
    % assemble net aerodynamic moment expressed in body frame
    Mab = [l;m;n] - cross(deltaCGb,Fab);    % cross(deltaCGb,Fab) -> analysis on the displacement of center of mass
end