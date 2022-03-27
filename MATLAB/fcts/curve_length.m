function len = curve_length(robot, curve, a, b, tol)
% This function computes the length of curve along the Riemannian manifold.
% The input curve must be parametric type: curve = @(t)...
%
% Parameters:
%   - robot:        a SerialLink manipulator
%   - a:            initial point of the curve
%   - b:            desired final point of the curve
%   - tol:          if the interal is smaller than tol, do not integrate
%
% Returns:
%   - tol:          curve length on the Riemannian manifold
%
% This code was adapted from the MATLAB code of Georgios Arvanitidis and
% Soren Hauberg, available at: https://github.com/georgiosarvanitidis/geometric_ml

%% Supply default arguments
if (nargin < 3)
    a = 0;
end % if
if (nargin < 4)
    b = 1;
end % if
if (nargin < 5)
    tol = 1e-6;
end % if

%% Integrate
if (abs(a - b) > 1e-6)
    if (~isnumeric(curve))
        len = integral(@(t) local_length(robot, curve, t), a, b, 'AbsTol', tol);
    end % if
end % if

end % function

%% The infinitesimal local lenght at a point c(t).
function d = local_length(robot, curve, t)
[q, dq] = curve(t); % [DxT, DxT]
nbDOFs = size(q,1);
N = size(q,2);
G = zeros(N, nbDOFs, nbDOFs);
for n=1:N
    G(n,:,:) = robot.inertia(q(:,n)');
end
v1 = squeeze(sum(bsxfun(@times, G, dq.'), 2)); % TxD
d = sqrt(sum(dq .* v1.', 1)); % Tx1
end % function
