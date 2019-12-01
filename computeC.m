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

% Date tested:    01/12/2019
% Tested by:      Alexander Gill
% Test procedure: 
% The spin 3 trajectory was used because it was the same as spin 2 except
% with a centrifugal pseudoforce on the end effector. This allows the
% centrifugal compensation to be tested in isolation. 
% Spin 5 was used to test compensation of gyroscopic forces. 
% Hold was used to check this code hasn't introduced new problems.
% Results:        
% The error was greatly reduced by enabling the output of this file,
% compared to when C was zero. 'Hold' worked as before.

% initialise empty vector of torques
C = zeros(1,6);

% loop over each link in 4D to get a value for each link
for n = 1:6
    for k = 1:6
        for j = 1:6
            
            % summation from i = 1 to N
            for i = 1:6
                
                % get transform derivatives
                Uij  = computeUij(T,i,n);
                Uijk = computeUijk(T,i,j,k);
                
                % evaluate restorative torque for this joint n
                C(n) = C(n) +...
                    trace(Uijk * J(:,:,i) * Uij') * qt(k) * qt(j);
            end
        end
    end
end
