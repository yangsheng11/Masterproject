function [ curve, logmap, len, solution ] = compute_geodesic_ivp(robot, q0, dq0)
% This function computes the shortest path between two given points on a
% Riemannian manifold. The utilized method is MATLAB's 'bvp5c'.
%
% Parameters:
%   - robot:        a SerialLink manipulator
%   - q0:           initial point of the curve
%   - qd:           desired final point of the curve
%   - init_curve:   starting solution for the parametric curve
%
% Returns:
%   - curve:        the parametric shortest curve c(t):[0,1]->M
%   - logmap:       logarithm map Log_q0(qd)
%   - len:          the curve length
%   - failed:       a boolean, if solver failed this value is true
%   - solution:     the struct containing the details of the solution
%
% This code was adapted from the MATLAB code of Georgios Arvanitidis and
% Soren Hauberg, available at: https://github.com/georgiosarvanitidis/geometric_ml


%% Format input
q0 = q0(:); % nbDOFsx1
dq0 = dq0(:); % nbDOFsx1
D = numel(q0);

%% Set up ODE system
odefun = @(x, y) first_order_geodesic_ode(robot, y);

%% Set up the options
options = bvpset('Vectorized', 'on', ...
    'RelTol', 5, ...
    'NMax', 100);

%% Initial point
init = [q0; dq0];

%% Solve the ODE
tic;
solution = ode45(odefun, [0 1], init);
solution.time_elapsed = toc;

%% Provide the output
curve = @(t) evaluate_solution(solution, t, 1);
logmap = solution.y((D+1):end, 1);

if (nargout > 1)
    len = curve_length(robot, curve);
    logmap = len * logmap / norm(logmap);
end % if


end % function

%% Additional functions
function bc = boundary(p0, p1, p0_goal, p1_goal)
D = numel(p0_goal);
d1 = p0(1:D) - p0_goal(:);
d2 = p1(1:D) - p1_goal(:);
bc = [d1; d2];
end % function

function [c, dc] = evaluate_solution (solution, t, t_scale)
  cdc = deval (solution, t*t_scale); % 2DxT
  D  = size (cdc, 1) / 2;
  c  = cdc (1:D, :).'; % TxD
  dc = cdc ((D+1):end, :).' * t_scale; % TxD
  c = c.'; dc = dc.';
end % function


function [c, dc] = evaluate_failed_solution(p0, p1, t)
t = t(:); % 1xT
c = (1 - t) * p0.' + t * p1.'; % TxD
dc = repmat ((p1 - p0).', numel(t), 1); % TxD
c = c.'; dc = dc.';
end % function

function state = init_solution(p0, p1, t)
[c, dc] = evaluate_failed_solution(p0, p1, t);
state = cat(1, c, dc); % 2DxT
end % function

function state = init_solution_given(solution, t)
[c, dc] = solution(t);
state = cat(1, c, dc); % 2DxT
end % function
