function T=computeT(q)
% computeT.m
% T = computeT(q)
% T is the 4x4x6 array of homogeneous transform matrices for all 6 links
%   e.g. T(:,:,3) is the matrix T23 (not T03!)
% q is a 6x1 vector of joint angles for the current time step, in radians

% Unit information:
% ME40331 Robotics Engineering, University of Bath
% Dynamics and Control lab 2016/2017
% Dr. Jon du Bois

%--------------------------------------------------------------------------
% DO NOT MOFIFY ABOVE THIS LINE!!!! YOUR CODE GOES BELOW THIS LINE.
%--------------------------------------------------------------------------  

%% hard-coded DH parameters
% thetas are available in the parameter 'q'
dValues = [0.3  0    0    0.2  0    0.1];
aValues = [0    0.2  0    0    0    0  ];
alphas  = [pi   0    pi  -pi   pi   0  ];

%% generate T matrices
% initialise empty T
T = zeros(4,4,6);

% loop over each T matrix
for i = 1:6
    % get parameters for ith link
    theta = q(i);
    d     = dValues(i);
    a     = aValues(i);
    alpha = alphas(i);
    
    % set ith T matrix
    T(:,:,i) = ...
[cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);...
 sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);...
 0,          sin(alpha),             cos(alpha),            d;...
 0,          0,                      0,                     1];
end
