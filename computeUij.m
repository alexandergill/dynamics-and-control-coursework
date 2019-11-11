function U=computeUij(T,i,j)
% computeUij.m
% U=computeUij(T,i,j)
% U is a 4x4 matrix describing the derivative of the transform from the
%   base frame to the ith link with respect to the jth generalised
%   coordinate.
% T is a 4x4x6 array of link transform matrices for the robot for the
%   current time step.
% i defines the link
% j defines the generalised coordinate

% Unit information:
% ME40331 Robotics Engineering, University of Bath
% Dynamics and Control lab 2016/2017
% Dr. Jon du Bois

%--------------------------------------------------------------------------
% DO NOT MOFIFY ABOVE THIS LINE!!!! YOUR CODE GOES BELOW THIS LINE.
%--------------------------------------------------------------------------   

U=zeros(4,4);
