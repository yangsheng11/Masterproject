function y = first_order_geodesic_ode(robot, qdq)
% This function computes the first order ODE to solve to obtain a geodesic.
% 
% Parameters:
%   - robot:    a SerialLink manipulator
%   - qdq:      concatenated joint positions and velocities [q;dq]
%               2*nbDOFs
% 
% Returns:
%   - y:        ODE equation [dq;ddq]

nbDOFs = size(qdq, 1) / 2;

% Decompose state in position q and velocity dq
q = qdq(1:nbDOFs,:);
dq = qdq(nbDOFs+1:end,:);

% Compute Euclidean acceleration corresponding to the geodesic
ddq = geodesic_ode(robot, q, dq);

% ODE
y = [dq; ddq];
end 
