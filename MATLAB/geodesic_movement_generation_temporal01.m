function geodesic_movement_generation_temporal01
% This function generates a geodesic motion of a planar robot. The
% configuration space of the robot is represented by a Riemannian manifold,
% whose Riemannian metric is given by the mass-inertia matrix of the robot.
% The geodesic is generated from an initial configuration and initial
% velocity by solving a geodesic ODE. The geodesic pathway is then modified
% to include a desired initial and final metric velocity along the geodesic
% pathway, as most functional motions require acceleration/deceleration.
% The temporal part is specified by minimizing the acceleration along the
% geodesic pathway. See also: "A Riemannian geometry theory of human 
% movement: the geodesic synergy hypothesis", Neilson et al, Human 
% Movement Science 2015.

% First run 'startup_rvc' from the robotics toolbox

addpath('./fcts/');

%% Create robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Robot parameters 
nbDOFs = 4; %Nb of degrees of freedom
armLength = 4; % Links length

% Robot
% Define mass of links
m_links = [1;1;1;1]; % [m1; m2; ...]
I_links = [0 0 1; 0 0 1;0 0 1;0 0 1;]; % [diag(I1); diag(I2); ...]
for n=1:nbDOFs
    Ln = Link('d', 0, 'a', armLength, 'alpha', 0, 'm', m_links(n), 'r', ...
        [-0.5 0 0], 'I', I_links(n,:), 'B', 0, 'G', 0, 'Jm', 0, 'standard');
    if n == 1
        Links = Ln;
    else
        Links = cat(1, Links, Ln);
    end
end
robot = SerialLink(Links);

% Symbolic values
q = sym('q', [1 nbDOFs]);	% Symbolic robot joints
J = robot.jacob0(q'); % Symbolic Jacobian
I = robot.inertia(q); % Mass-inertia matrix of the system - This is also the Riemannian metric! 
% cI = robot.cinertia(q); % Cartesian mass-inertia matrix

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
    last_link_virtual = Link('d', 0, 'a', robot.a(n) + Links(n).r(1), 'alpha', 0);
    Links_virtual = cat(1, robot.links(1:n-1), last_link_virtual);
    virtualRobots{n} = SerialLink(Links_virtual);
end

%% Geodesic motion generation including temporal aspect
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initial conditions
q0 = [pi/2 ; -pi/6; pi/3; pi/6]; % Initial robot configuration 
dq0 = [5.; 5.; 5.; 5.]; % Initial robot velocity
dq0 = dq0./norm(dq0);

[ curve, logmap, len, solution ] = compute_geodesic_ivp(robot, q0, dq0);

% Compute the temporal parametrization, so that initial and final
% velocities are equal to 0
nbPoints = 100;
s0 = 0;
s1 = len;
ds0 = 0;
ds1 = 0;
T = 1;
t = linspace(0, T, nbPoints);
c1 = -12/T^3 * (s1-s0) + 6/T^2 * (ds1 + ds0);
c2 = 6/T^2 * (s1 -s0) - 2/T * (ds1 + 2*ds0);
c3 = ds0;
c4 = s0;
for it=1:nbPoints
    s(it) = c1 * t(it)^3 / 6 + c2 * t(it)^2 / 2 + c3 * t(it) + c4;
end
% Normalize s between 0 and 1
s = s / len;
s(1) = s(1) + 1e-8;
s(end) = s(end) - 1e-8;

% Compute joint position and velocity along the trajectory (velocity
% computed based on position 
[qt_temporal, ~] = curve(s); % DxN
dqt_temporal = (qt_temporal(:, 2:end) - qt_temporal(:, 1:end-1)) * nbPoints;
dqt_temporal(:, 100) = zeros(nbDOFs, 1);

% Compute inertia along the trajectory
Gt_temporal = zeros(nbDOFs,nbDOFs,nbPoints);
for it=1:nbPoints
    % Compute Jacobian for each link
    Ji = cell(nbDOFs,1);
    for i = 1:nbDOFs
        Ji{i} = [virtualRobots{i}.jacob0(qt_temporal(1:i,it)) zeros(6,nbDOFs-i)];
    end
    
    % Compute inertia matrix
%     Gt = robot.inertia(qt'); % Current mass-inertia matrix == Riemannian metric
    Gt_temporal(:,:,it) = compute_inertia(Ji,Mi); % Current mass-inertia matrix == Riemannian metric
end

% Compute velocity norm
for it=1:nbPoints
    dq_geodesic_norm_temporal(it) = dqt_temporal(:,it)'*Gt_temporal(:,:,it)*dqt_temporal(:,it);
    dq_linear_norm(it) = dqt_temporal(:,it)'*dqt_temporal(:,it);
end

%% Geodesic motion generation (spatial part only)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute original geodesic path (constant velocity)
t = linspace(0, 1, nbPoints);
[qt_spatial, dqt_spatial] = curve(t); % DxN

% Compute inertia along the trajectory
Gt_spatial = zeros(nbDOFs,nbDOFs,nbPoints);
for it=1:nbPoints
    % Compute Jacobian for each link
    Ji = cell(nbDOFs,1);
    for i = 1:nbDOFs
        Ji{i} = [virtualRobots{i}.jacob0(qt_temporal(1:i,it)) zeros(6,nbDOFs-i)];
    end
    
    % Compute inertia matrix
%     Gt = robot.inertia(qt'); % Current mass-inertia matrix == Riemannian metric
    Gt_spatial(:,:,it) = compute_inertia(Ji,Mi); % Current mass-inertia matrix == Riemannian metric
end

% Compute velocity norm
for it=1:nbPoints
    dq_geodesic_norm_spatial(it) = dqt_spatial(:,it)'*Gt_spatial(:,:,it)*dqt_spatial(:,it);
end

%% Plots
% Plot robot pose evolution
figure('position',[10 10 900 900],'color',[1 1 1]);
hold on;
p_spatial = [];
p_temporal = [];
for it = 1:10:nbPoints
	colTmp = [1,.8,1] - [.8,.8,.8] * (it)/nbPoints;
    p_spatial = [p_spatial; plotArm(qt_spatial(:,it), ones(nbDOFs,1)*armLength, [0; 0; it*0.1], .1, colTmp)];
	colTmp = [.9,.9,.9] - [.8,.8,.8] * (it)/nbPoints;
    p_temporal = [p_temporal; plotArm(qt_temporal(:,it), ones(nbDOFs,1)*armLength, [0; 0; it*0.1], .1, colTmp)];
end
p_spatial = [p_spatial; plotArm(qt_spatial(:,it), ones(nbDOFs,1)*armLength, [0; 0; it*0.1], .1, [.2,.0,.2])];
p_temporal = [p_temporal; plotArm(qt_temporal(:,it), ones(nbDOFs,1)*armLength, [0; 0; it*0.1], .1, [.1,.1,.1])];

axis equal
set(gca,'xtick',[],'ytick',[])
xlabel('$x_1$','fontsize',40,'Interpreter','latex'); ylabel('$x_2$','fontsize',40,'Interpreter','latex');

% Plot velocity along the trajectories
figure('position',[10 10 800 400],'color',[1 1 1]); hold on;
left_color = [0 0 0];
right_color = [0 0 .7];
% set(fig,'defaultAxesColorOrder',[left_color; right_color]);
set(gca,'fontsize',12);
xlabel('$t$','fontsize',20,'Interpreter','latex');
yyaxis left
plot([0:nbPoints-1]./nbPoints, dq_geodesic_norm_spatial, '-','color',[0 0 0],'Linewidth',3);
plot([0:nbPoints-1]./nbPoints, dq_geodesic_norm_temporal, '-','color',[0.5 0.5 0.5],'Linewidth',3);
ylabel('$\|\dot{\mathbf{q}}\|_{\mathbf{G}}$','fontsize',20,'Interpreter','latex');
yyaxis right
plot([0:nbPoints-1]./nbPoints, dq_linear_norm, '-','color',[0 0 .7],'Linewidth',3);
ylabel('$\|\dot{\mathbf{q}}\|$','fontsize',20,'Interpreter','latex');
ax = gca;
ax.YAxis(1).Color = left_color;
ax.YAxis(2).Color = right_color;

end