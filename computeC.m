function C=computeC(J,T,qt)
% computeC.m
% C=computeC(J,T,qt)
% C  is a 6x1 vector of torques due to centrifugal, centripetal, Coriolis
%    and precessional forces for the current timestep.
% J  is a 4x4x6 array of pseudo-inertia matrices for the robot links.
% T  is a 4x4x6 array of link transform matrices for the robot for the
%    current time step.
% qt is a 6x1 vector of joint velocities (angluar velocities) for the
%    current time step, in radians/s.

% Unit information:
% ME40331 Robotics Engineering, University of Bath
% Dynamics and Control lab 2016/2017
% Dr. Jon du Bois

%--------------------------------------------------------------------------
% DO NOT MOFIFY ABOVE THIS LINE!!!! YOUR CODE GOES BELOW THIS LINE.
%--------------------------------------------------------------------------

% Date tested:    -/-/2018
% Tested by:      Joe Bloggs
% Test procedure: 
% Results:        

C=[0;0;0;0;0;0];
