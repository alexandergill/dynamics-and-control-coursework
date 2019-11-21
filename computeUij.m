function U=computeUij(T,i,j)
% computeUij.m
% U=computeUij(T,i,j)
% U is a 4x4 matrix describing the derivative of the transform from the
%   base frame to the ith link with respect to the jth generalised
%   coordinate.
% T is a 4x4x6 array of link transform matrices for the robot for the
%   current time step.
% i defines the link
% j defines the generalised coordinate

% Unit information:
% ME40331 Robotics Engineering, University of Bath
% Dynamics and Control lab 2016/2017
% Dr. Jon du Bois

%--------------------------------------------------------------------------
% DO NOT MOFIFY ABOVE THIS LINE!!!! YOUR CODE GOES BELOW THIS LINE.
%--------------------------------------------------------------------------   

if i < j
    U = zeros(4);
else
    % initialise U to an identity matrix to allow its value to be assigned
    % by multiplication
    U = eye(4);
    for index = 1:i
        % if this term is the jth term, multiply by the derivative, otherwise
        % multiply by the T matrix for this term
        if index == j            
            U = U * derivative(T(:,:,index));
        else
            U = U * T(:,:,index);
        end
    end
end
end

function dTdq = derivative(T)
% derivative returns the derivative with respect to q of the T matrix
% this is calculated by multiplying a matrix 'mat' with the T matrix

    % create matrix and compute derivative
    mat = [0 -1  0  0;...
           1  0  0  0;...
           0  0  0  0;...
           0  0  0  0];
    dTdq = mat * T;
end
