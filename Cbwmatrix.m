% Cbwmatrix.m
% function to calculate rotation matrix from relative wind frame to body frame
% usage
% Cbw = Cbwmatrix(alpha,beta)
% where 
%   alpha : angle of attack (rad)
%   beta : angle of sideslip (rad)
%   Cbw : Rotation matrix from relative wind frame to body frame
function Cbw = Cbwmatrix(alpha,beta)
    % Calculate cosine and sine of alpha an beta
    calpha = cos(alpha);
    salpha = sin(alpha);
    cbeta = cos(beta);
    sbeta = sin(beta);

    Cbw =[calpha*cbeta -calpha*sbeta -salpha;
        sbeta cbeta 0;
        salpha*cbeta -salpha*sbeta calpha];
end