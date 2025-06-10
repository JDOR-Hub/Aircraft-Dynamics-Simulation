%faircraft.m
%function to implement equations of motion for a fixed wing aircraft
%usage
%   [xdot,y] = faircraft(t,x,delta,Vwe,deltaCGb, aircraft)
% where
% t :   tiempo (s)
% x :   [pe;Phi;Vb;omegab] :    state vector
%       with:
%       pe : Position of aircraft CG respect to earth expressed in eart
%       frame (m)
%       Phi     : [phi;theta;psi] : Euler angles (rad)
%       Vb      : [u;v;w] aircraft velocity respect to earth expressed in
%                 body frame (m/s)
%       omegab  : [p;q;r]aircraft angular velocity respect to earth expressed in
%                 body frame (rad/s)
% delta : [deltat;deltaf;ih;deltae;deltaa;deltar]: aircraft controls vector
%        with:
%       deltat : Propulsion system control (0<=deltat<=1)       
%       deltaf : flap (rad)
%       ih :  Horizontal tail incidence (rad)
%       deltae : elevador (rad) 
%       deltaa : Aileron (rad)
%       deltar : rudder (rad)
%       deltaaero : [deltaf;ih;deltae;deltaa;deltar] : aircraft aerodynamic
%                   control vector
%       
% Vwe : wind velocity expressed in earth frame (m/s)
% deltaCGb : relative CG position respect to nominal CG expressed in body
%            frame (m)
% aircraft : aircraft data structure 
% xdot :   [pedot;Phidot;Vbdot;omegabdot] :    derivative of state vector
%       with:
%       pedot : Ve : derivative of position of aircraft CG respect to earth expressed in eart
%       frame (m/s)
%       Phidot  : [phidot;thetadot;psidot] : derivative of euler angles (rad/s)
%       Vbdot   : [udot;vdot;wdot] derivative of aircraft velocity respect to earth expressed in
%                 body frame (m/s^2)
%       omegabdot: [pdot;qdot;rdot] : derivative of aircraft angular velocity respect to earth expressed in
%                 body frame (rad/s^2) 
% y = [V;alpha;beta] : airspeed (m/s), angle of attack (rad), angle of
%                     sideslip (rad)


function [xdot,y] = faircraft(t,x,delta,Vwe,deltaCGb, aircraft)
    global Vbdot
    % extract components of x an delta
    pe = x(1:3,1);
    h = -pe(3);
    Phi = x(4:6,1);
    Vb = x(7:9,1);
    omegab = x(10:12,1);
    deltat = delta(1,1);
    deltaaero = delta(2:6,1);
    
    % evaluate translational kinematics
    
    Cbe = DCM(Phi);

    pedot = Cbe'*Vb;
    
    % evaluate rotationak kinematics
    Phidot = H(Phi)*omegab;
    
    % calculate forces and moments
    % weight
    Ge = [0; 0; aircraft.g];
    Gb = Cbe*Ge;
    Wb = aircraft.m*Gb;
    
    % calculate relative velocity
    Vrelb = Vb-Cbe*Vwe;
    
    % calculate airspeed, angle of attack and sideslid angle
    V = norm(Vrelb);
    alpha = atan(Vrelb(3)/Vrelb(1));
    beta = asin(Vrelb(2)/V);
    
    % calculate derivative of relative velocity
    Vrelbdot = Vbdot + cross(omegab,Cbe*Vwe);
    
    % calculate derivate of angle of attack
    alphadot = (Vrelbdot(3) * Vrelb(1) - Vrelbdot(1)*Vrelb(3))/(Vrelb(1)^2 + Vrelb(3)^2);
    
    % calculate atmosphere parameters
    [rho,P,T,a] = atmosphere(h);
    
    % calculate dynamic pressure
    qbar = rho*V^2/2;
    
    % calculate mach number
    M = V/a;
    
    % calculate aerodunamic forces and moments
    [Fab,Mab] = aerodynamics(V,alpha,beta,alphadot,omegab,deltaaero,qbar,M,deltaCGb,aircraft);
    
    % calculate propulsion system forces and moments
    [Ftb,Mtb] = propulsion(deltat,deltaCGb,aircraft);

    % calculate net force expressed in body frame
    Fnetb = Wb + Fab + Ftb;

    % evaluate translational dynamics
    Vbdot = Fnetb/aircraft.m - cross(omegab,Vb);

    % calculate net moment expressed in body frame
    Mnetb = Mab + Mtb;

    % evaluate rotational dynamics
    omegabdot = aircraft.Ibinv*(Mnetb - cross(omegab,aircraft.Ib*omegab));
    
    % assemble xdot and y
    xdot = [pedot;Phidot;Vbdot;omegabdot];
    y = [V;alpha;beta];

end








