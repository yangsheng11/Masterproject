function geodesic_movement_generation01
% This function generates a geodesic motion of a planar robot. The
% configuration space of the robot is represented by a Riemannian manifold,
% whose Riemannian metric is given by the mass-inertia matrix of the robot.
% The geodesic is generated from an initial configuration and initial
% velocity by solving a geodesic ODE. See also: "A Riemannian geometry
% theory of human movement: the geodesic synergy hypothesis", Neilson et
% al, Human Movement Science 2015.

% First run 'startup_rvc' from the robotics toolbox

addpath('./fcts/');

%% Create robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Robot parameters 
nbDOFs = 2; %Nb of degrees of freedom
armLength = 1; % Links length

% Robot
% Define mass of links
m_links = [1;1]; % [m1; m2; ...]
I_links = [0 0 1; 0 0 1]; % [diag(I1); diag(I2); ...]
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

%% Geodesic motion generation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initial conditions
q0 = [pi/2 ; -pi/6]; % Initial robot configuration 
dq0 = [5.; 5.]; % Initial robot velocity
dq0 = dq0./norm(dq0);

[ curve, logmap, len, solution ] = compute_geodesic_ivp(robot, q0, dq0);

% Compute joint position and velocity along the trajectory
nbPoints = 100;
t = linspace(0, 1, nbPoints);
[qt, dqt] = curve(t); % DxN

% Compute inertia along the trajectory
Gt = zeros(nbDOFs,nbDOFs,nbPoints);
Gt_grad = zeros(nbDOFs,nbDOFs,nbDOFs,nbPoints);
for it=1:nbPoints
    % Compute Jacobian for each link
    Ji = cell(nbDOFs,1);
    for i = 1:nbDOFs
        Ji{i} = [virtualRobots{i}.jacob0(qt(1:i,it)) zeros(6,nbDOFs-i)];
    end
    
    % Compute inertia matrix
%     Gt = robot.inertia(qt'); % Current mass-inertia matrix == Riemannian metric
    Gt(:,:,it) = compute_inertia(Ji,Mi); % Current mass-inertia matrix == Riemannian metric
    Gt_grad(:,:,:,it) = compute_joint_derivative_inertia(Ji, Mi);
end

% Compute velocity norm
for it=1:nbPoints
    dq_geodesic_norm(it) = dqt(:,it)'*Gt(:,:,it)*dqt(:,it);
    dq_linear_norm(it) = dqt(:,it)'*dqt(:,it);
end

%% Plots
% Plot robot pose evolution
figure('position',[10 10 900 900],'color',[1 1 1]);
hold on;
p = [];
for it = 1:10:nbPoints
	colTmp = [1,1,1] - [.8,.8,.8] * (it)/nbPoints;
	p = [p; plotArm(qt(:,it), ones(nbDOFs,1)*armLength, [0; 0; it*0.1], .2, colTmp)];
end

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
plot([0:nbPoints-1]./nbPoints, dq_geodesic_norm, '-','color',[0 0 0],'Linewidth',3);
ylabel('$\|\dot{\mathbf{q}}\|_{\mathbf{G}}$','fontsize',20,'Interpreter','latex');
yyaxis right
plot([0:nbPoints-1]./nbPoints, dq_linear_norm, '-','color',[0 0 .7],'Linewidth',3);
ylabel('$\|\dot{\mathbf{q}}\|$','fontsize',20,'Interpreter','latex');
ax = gca;
ax.YAxis(1).Color = left_color;
ax.YAxis(2).Color = right_color;

end
