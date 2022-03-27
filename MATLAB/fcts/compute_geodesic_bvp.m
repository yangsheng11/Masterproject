function [ curve, logmap, len, failed, solution ] = compute_geodesic_bvp(robot, q0, q1, init_curve)
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
q1 = q1(:); % nbDOFsx1
D = numel(q0);

%% Set up boundary conditions
bc = @(a, b) boundary(a, b, q0, q1);

%% Set up ODE system
odefun = @(x, y) first_order_geodesic_ode(robot, y);

%% ODE Jacobian
odejac = @(x,y) jacobian_geodesic_ode(robot, y);

%% Set up the options
options = bvpset('Vectorized', 'on', ...
    'RelTol', 5, ...
    'NMax', 100, ...
    'FJacobian',odejac);

%% Initial guess
% TODO check if we can give a better initial guess than the linear
% interpolation
if (nargin < 4)
    x_init = linspace(0, 1, 10); % 1xT
    y_init = @(t) init_solution(q0, q1, t);
    init = bvpinit(x_init, y_init);
else
    x_init = linspace(0, 1, 10); % 1xT
    y_init = @(t) init_solution_given(init_curve, t);
    init = bvpinit(x_init, y_init);
end % if

%% Try to solve the ODE
tic;
solution = bvp5c(odefun, bc, init, options);
failed = false;
% try
%     %         solution = bvp4c(odefun, bc, init, options);
%     solution = bvp5c(odefun, bc, init, options);
%     if (isfield(solution, 'stats') && isfield(solution.stats, 'maxerr'))
%         if (isfield(options, 'RelTol') && isscalar(options.RelTol))
%             reltol = options.RelTol;
%         else
%             reltol = 1e-3;
%         end % if
%         failed = (solution.stats.maxerr > reltol);
%     else
%         failed = false;
%     end % if
% catch
%     disp('Geodesic solver (bvp5c) failed!');
%     failed = true;
%     solution = [];
% end % try
solution.time_elapsed = toc;

%% Provide the output
if (failed)
    curve = @(t) evaluate_failed_solution(q0, q1, t);
    logmap = (q1 - q0);
else
    curve = @(t) evaluate_solution(solution, t, 1);
    logmap = solution.y((D+1):end, 1);
end % if
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
