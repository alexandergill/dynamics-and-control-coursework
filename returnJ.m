function J=returnJ
% returnJ.m
% J=returnJ()
% J  is a 4x4x6 array of pseudo-inertia matrices for the robot links.

% Unit information:
% ME40331 Robotics Engineering, University of Bath
% Dynamics and Control lab 2016/2017
% Dr. Jon du Bois

%--------------------------------------------------------------------------
% DO NOT MOFIFY ABOVE THIS LINE!!!! YOUR CODE GOES BELOW THIS LINE.
%--------------------------------------------------------------------------     

J=zeros(4,4,6);                    % initialise pseudo inertia matrix array
