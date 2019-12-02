function J=returnJ
% returnJ.m
% J=returnJ()
% J  is a 4x4x6 array of pseudo-inertia matrices for the robot links.

% Unit information:
% ME40331 Robotics Engineering, University of Bath
% Dynamics and Control lab 2016/2017
% Dr. Jon du Bois

%--------------------------------------------------------------------------
% DO NOT MODIFY ABOVE THIS LINE!!!! YOUR CODE GOES BELOW THIS LINE.
%--------------------------------------------------------------------------     

%% hard-coded parameters for the robot
density = 7850;
lengths = [0.3   0.2  0.03 0.2  0.03 0.1]; % x dimensions
widths  = [NaN   0.03 0.03 NaN  0.03 NaN]; % y dimensions
depths  = [NaN   0.01 0.03 NaN  0.03 NaN]; % z dimensions
radii   = [0.02  NaN  NaN  0.01 NaN  0.01];
x_bars  = [0    -0.1  0    0    0    0];
y_bars  = [-0.15 0    0    0.1  0    0];
z_bars  = [0     0    0    0    0   -0.05];

%% get masses of each link
masses = zeros(1, 6);

% 1, 4, and 6 are cylinders
for i = [1 4 6]
    masses(i) = pi * radii(i)^2 * lengths(i) * density;
end

% 2, 3, and 5 are cuboids
for i = [2 3 5]
    masses(i) = depths(i) * widths(i) * lengths(i) * density;
end

%% get moments of inertia for each frame
% I is a 3x6 array of moments of inertia. Each 3x3 element contains the
% moments in the form:
%
%  Ixx Iyy Izz   (note that Izx = Ixz = 0 and Ixy = Iyx = 0) 
%
I = zeros(3,6);

% links 1, 4, and 6 are cylinders
% need to remap axes because the frame alignment is not the same for each
% cylinder
[I(2,1),I(1,1),I(3,1)] = calculateCylinderI(lengths(1), radii(1), masses(1));
[I(2,4),I(1,4),I(3,4)] = calculateCylinderI(lengths(4), radii(4), masses(4));
[I(3,6),I(2,6),I(1,6)] = calculateCylinderI(lengths(6), radii(6), masses(6));

% links 2, 3, and 5 are cuboids
% no need to remap axes because x axis is along the link for all three
% cuboids
for i = [2 3 5]
    % get dimensions of this cuboid
    x = lengths(i); y = widths(i); z = depths(i);
    xbar = x_bars(i); ybar = y_bars(i); zbar = z_bars(i);
    % get moments of inertia
    [I(1,i),I(2,i),I(3,i)] = calculateCuboidI(x,y,z,xbar,ybar,zbar,masses(i));
end

%% calculate J matrix for each link
J = zeros(4,4,6);  

for i = 1:6
    % get moments of inertia for this link
    Ixx = I(1,i); Iyy = I(2,i); Izz = I(3,i);
    
    % get axial offsets for this link
    xbar = x_bars(i);
    ybar = y_bars(i);
    zbar = z_bars(i);
    
    % calculate J matrix elements
    J(1,1,i) = 0.5*(-Ixx+Iyy+Izz); % \
    J(2,2,i) = 0.5*(Ixx-Iyy+Izz);  %  | inertia terms
    J(3,3,i) = 0.5*(Ixx+Iyy-Izz);  % /
    
    % set final column of J matrix
    J(:,4,i) = [xbar; ybar; zbar; 1] * masses(i);
    
    % bottom row is the transpose of the final column
    J(4,:,i) = J(:,4,i)';
end

end

function [Ixx, Iyy, Izz] = calculateCylinderI(length, radius, mass)
% calculates the moments of intertia of a cylinder given that the x axis is
% aligned with the vertical axis and the axes are offset from the centre by
% axialOffset

% each axis set is at the end of the cylinder
offset = length / 2;

% get Ixx = 1/2 m r^2
Ixx = 0.5 * mass * radius^2;

% get Iyy = Iyy = 1/12 m (3r^2 + h^2)
Iyy = (1/12) * mass * (3 * radius^2 + length^2);

% account for offset in axis alignment using parallel axis theorem
Iyy = Iyy + ( mass * offset^2 );
Izz = Iyy;
end

function [Ixx, Iyy, Izz] = calculateCuboidI(x,y,z,xbar,ybar,zbar,m)
    % calculate each moment of inertia including the parallel axis theorem
	Ixx = (1/12)*m*(y^2 + z^2) + m*(ybar^2 + zbar^2);
    Iyy = (1/12)*m*(x^2 + z^2) + m*(xbar^2 + zbar^2);
    Izz = (1/12)*m*(x^2 + y^2) + m*(xbar^2 + ybar^2);
end