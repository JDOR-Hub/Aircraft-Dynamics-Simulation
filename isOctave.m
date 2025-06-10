% function to check wether the mcode is running in Octave
%
% usage
%   y = isOctave()
% where
%   y=1 when running in Octave and y=0 when running in Matlab
%
% Author:
%   Luis Benigno Gutierrez Zea
%   luis.gutierrez@upb.edu.co
%

function y = isOctave()
  y = exist('OCTAVE_VERSION','builtin')==5;
end
