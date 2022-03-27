function jac = jacobian_geodesic_ode(robot, qdq)
% This function computes the Jacobian of the first order ODE to solve to obtain a geodesic. 
% This may be used to improve the performance of the ODE solver.
% 
% Parameters:
%   - robot:    a SerialLink manipulator
%   - qdq:      concatenated joint positions and velocities [q;dq]
%               2*nbDOFs
% 
% Returns:
%   - jac:      ODE Jacobian [dq/q dq/dq; ddq/q ddq/dq]

nbDOFs = size(qdq, 1) / 2;

% Decompose state in position q and velocity dq
q = qdq(1:nbDOFs,:);
dq = qdq(nbDOFs+1:end,:);

% Compute mass-inertia matrix of each link
Mi = zeros(6, 6, nbDOFs);
for n=1:nbDOFs
    b = zeros(3);
    Mi(:,:,n) = [robot.links(n).m.*eye(3) robot.links(n).m.*b'; ...
        robot.links(n).m.*b robot.links(n).I];
end

% Create virtual robots, to compute the Jacobian of each link after
virtualRobots = cell(nbDOFs,1);
for n=1:nbDOFs
    last_link_virtual = Link('d', 0, 'a', robot.a(n) + robot.links(n).r(1), 'alpha', 0);
    Links_virtual = cat(1, robot.links(1:n-1), last_link_virtual);
    virtualRobots{n} = SerialLink(Links_virtual);
end

% Compute Jacobian for each link
Ji = cell(nbDOFs,1);
for i = 1:nbDOFs
    Ji{i} = [virtualRobots{i}.jacob0(q(1:i,:)) zeros(6,nbDOFs-i)];
end
    
% Compute the derivative of the acceleration of the geodesic spray field
[df2dc, df2dv] = compute_derivative_acceleration_geodesic_spray_field(dq, Ji, Mi);

% Compute the ODE Jacobian
jac = [zeros(nbDOFs,nbDOFs), eye(nbDOFs); df2dc, df2dv];

end