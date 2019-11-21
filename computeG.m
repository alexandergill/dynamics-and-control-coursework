function G=computeG(T)
% computeG.m
% G=computeG(T)
% G is a 6x1 vector of torques due to gravity forces for the current
%   timestep.
% T is the 4x4x6 array of link transform matrices for the robot for the
%   current time step.

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

%% hard-coded parameters for this robot
density = 7850;
radii   = [0.02 NaN NaN 0.01 NaN 0.01];
lengths = [0.3 0.2 0.03 0.2 0.03 0.1];
thicknesses  = [NaN 0.01 0.03 NaN 0.03 NaN];
depths = [NaN 0.03 0.03 NaN 0.03 NaN];
masses = zeros(1, 6);
% links 1, 4, and 6 are cylinders
for i = [1 4 6]
    masses(i) = density * pi * radii(i).^2 * lengths(i);
end
% links 2, 3, and 5 are cuboids
for i = [2 3 5]
    masses(i) = density * lengths(i) * thicknesses(i) * depths(i);
end

r = [0    -0.15 0;...
     -0.1 0     0;...
     0    0     0;...
     0    0.1   0;...
     0    0     0;...
     0    0     -0.05];

g = [0;0;9.81;0];
%% summation of gravity terms

G=[0;0;0;0;0;0];

for n = 1:6
    for i = 1:6
        U = computeUij(T,i,n);
        mi = masses(i);
        ri = r(i,:);
        G(n) = G(n) - (mi * g' * U * ri);
    end
end

G=[0;0;0;0;0;0];
