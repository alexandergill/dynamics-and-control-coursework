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

% Date tested:    01/12/2019
% Tested by:      Alexander Gill
% Test procedure: 
% Compared error when Mqtt was zero vs when it was the output of this file.
% The following trajectories were used:
%
%    trajectory        reason
%  ----------------------------------------------------------------------
%    spin 1            Mqtt only handles forces due to acceleration, and
%                      this trajectory only has acceleration in one
%                      direction, allowing debugging of this file in
%                      isolation.
%    
%    spin 2            This trajectory only includes acceleration and
%                      gravity compensation. G is known to work, so this
%                      allows any problems to be attributed to this code.
%
%    hold              Check this code hasn't introduced new problems
%
% Results:
% When Mqtt was zero, large oscillation was observed for both spin
% trajectories, causing significant error. When Mqtt was the output of this
% file, the oscillation was significantly reduced and approximately zero.
% Some wobble was observed still in spin 2 but it was a lot less than
% without this implementation. Hold still worked as before.

%% initialise empty NxN mass matrix
Mnj  = zeros(6);

%% fill mass matrix
% loop over the robot's joints in 3D, because the mass matrix is 2D and
% requires summation in the third dimension
for n = 1:6
    for j = 1:6
        for i = 1:6
            
            % get transform derivatives
            Uij = computeUij(T,i,j);
            Uin = computeUij(T,i,n);
            
            % compute this element of the mass matrix
            Mnj(n,j) = Mnj(n,j) + trace(Uij * J(:,:,i) * Uin');
        end
    end
end

%% multiply mass matrix by accelerations to get restorative force
Mqtt = Mnj * qtt;
