function U=computeUijk(T,i,j,k)
% computeUijk.m
% U=computeUijk(T,i,j,k)
% U is a 4x4 matrix describing the second derivative of the transform from
%   the base frame to the ith link with respect to the jth and kth
%   generalised coordinates.
% T is a 4x4x6 array of link transform matrices for the robot for the
%   current time step.
% i defines the link
% j  defines the first generalised coordinate
% k  defines the second generalised coordinate

% Unit information:
% ME40331 Robotics Engineering, University of Bath
% Dynamics and Control lab 2016/2017
% Dr. Jon du Bois

%--------------------------------------------------------------------------
% DO NOT MOFIFY ABOVE THIS LINE!!!! YOUR CODE GOES BELOW THIS LINE.
%--------------------------------------------------------------------------      

% initialise empty U matrix
U = zeros(4);

% return zero matrix under these conditions
if i < j || i < k
    return
else
    % initialise U to identity matrix so its value can be assigned through
    % multiplication
    U = eye(4);
    
    % get product of terms from 1 to i
    for index = 1:i
        thisTerm = T(:,:,index);
        
        % differentiate terms if this iteration is j or k
        % differentiate twice if j = k
        if index == j
            thisTerm = derivative(thisTerm);
        end
        if index == k
            thisTerm = derivative(thisTerm);
        end
        
        % get product at this iteration
        U = U * thisTerm;
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
