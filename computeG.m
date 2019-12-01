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

% Date tested:    27/11/2019
% Tested by:      Alexander Gill
%% Test procedure:
% 1. Append the following line to the end of this file:
%    G=[0;0;0;0;0;0];
% 2. Run simulation using 'hold' trajectory
% 3. Save the resulting graph
% 4. Remove the above line from the end of this file
% 5. Run the same simulation
% 6. Compare the results
%
% The 'hold' trajectory was chosen because this tests the gravity
% compensation in isolation, without any movement to confuse the results.
%% Results:
% When G was set to zeros in step 1, the robot oscillated
% When G was set to the correct values in step 4, the robot did not
% oscillate.
%% Conclusion:
% The gravity compensation is working as expected.

%% hard-coded parameters for this robot
density =      7850;
radii   =      [0.02 NaN  NaN  0.01 NaN  0.01];
lengths =      [0.3  0.2  0.03 0.2  0.03 0.1 ];
thicknesses  = [NaN  0.01 0.03 NaN  0.03 NaN ];
depths =       [NaN  0.03 0.03 NaN  0.03 NaN ];

%% calculate masses for this robot
masses = zeros(1, 6); % initialise to zero
% links 1, 4, and 6 are cylinders
for i = [1 4 6]
    masses(i) = density * pi * radii(i).^2 * lengths(i);
end
% links 2, 3, and 5 are cuboids
for i = [2 3 5]
    masses(i) = density * lengths(i) * thicknesses(i) * depths(i);
end

%% r vectors
% this matrix's rows are the r vector for each link
r = [0    -0.15 0     1;...
     -0.1 0     0     1;...
     0    0     0     1;...
     0    0.1   0     1;...
     0    0     0     1;...
     0    0     -0.05 1];

g = [0;0;-9.81;0];

%% summation of gravity terms
G=[0;0;0;0;0;0]; % initialise to zero

% loop over links
for n = 1:6
    % create summation for this link by looping over links
    for i = 1:6
        % evaluate the equation terms for this link
        U = computeUij(T,i,n);
        mi = masses(i);
        ri = r(i,:)';
        G(n) = G(n) - (mi * g' * U * ri);
    end
end
