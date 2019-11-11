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

%% get moments of inertia for each frame
% I is a 3x3x6 array of moments of inertia. Each 3x3 element contains the
% moments in the form:
%
%  Ixx  Ixy  Ixz
%  Iyx  Iyy  Iyz
%  Izx  Izy  Izz   (note that Izx = Ixz and Ixy = Iyx) 
%
I = zeros(3,3,6);

% links 1, 4, and 6 are cylinders
for i = [1 4 6]
    Izz = 0.5 * m(i) * r(i).^2;
end

% links 2, 3, and 5 are cuboids

J=zeros(4,4,6);                    % initialise pseudo inertia matrix array
