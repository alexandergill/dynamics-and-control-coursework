function Mqtt=computeMqtt(J,T,qtt)
% computeMqtt.m
% Mqtt=computeMqtt(J,T,qtt)
% Mqtt is a 6x1 vector of torques due to joint acceleration forces for the
%      current timestep.
% J    is a 4x4x6 array of pseudo-inertia matrices for the robot links.
% T    is a 4x4x6 array of link transform matrices for the robot for the
%      current time step.
% qtt  is a 6x1 vector of joint accelerations (angluar accelerations) for
%      the current time step, in radians/s^2.

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

Mqtt=zeros(6,1);
