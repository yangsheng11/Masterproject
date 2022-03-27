function ddq = geodesic_ode(robot, q, dq)
% This function computes the second order ODE to solve to obtain a geodesic. 
% In order to be solved, this ODE needs to be transformed onto a first
% order ODE.
% 
% Parameters:
%   - robot:    a SerialLink manipulator
%   - q:        joint positions 
%   - dq:       joint velocities 
% 
% Returns:
%   - ddq:      joint accelerations

nbDOFs = size(robot.links,1);
N = size(q,2);

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

% Acceleration
ddq = zeros(nbDOFs,N);
for n=1:N
    % Compute Jacobian for each link
    Ji = cell(nbDOFs,1);
    for i = 1:nbDOFs
        Ji{i} = [virtualRobots{i}.jacob0(q(1:i,n)) zeros(6,nbDOFs-i)];
    end
    
    % Compute acceleration
    ddq(:,n) = compute_acceleration_geodesic_spray_field(dq(:,n), Ji, Mi);
end

% ODE
% y = [dq; ddq];

end