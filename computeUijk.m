function U=computeUijk(T,i,j,k)
% computeUijk.m
% U=computeUijk(T,i,j,k)
% U is a 4x4 matrix describing the second derivative of the transform from
%   the base frame to the ith link with respect to the jth and kth
%   generalised coordinates.
% T is a 4x4x6 array of link transform matrices for the robot for the
%   current time step.
% i defines the link
% j  defines the first generalised coordinate
% k  defines the second generalised coordinate

% Unit information:
% ME40331 Robotics Engineering, University of Bath
% Dynamics and Control lab 2016/2017
% Dr. Jon du Bois

%--------------------------------------------------------------------------
% DO NOT MOFIFY ABOVE THIS LINE!!!! YOUR CODE GOES BELOW THIS LINE.
%--------------------------------------------------------------------------      

U=zeros(4,4);
