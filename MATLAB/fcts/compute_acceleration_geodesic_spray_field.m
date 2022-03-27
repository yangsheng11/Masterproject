function f2 = compute_acceleration_geodesic_spray_field(dq, Ji, Mi)
% This function computes the acceleration component of the geodesic spray
% field for a given configuration and velocity of the robot. When following
% a geodesic trajectory on the configuration Riemannian manifold, the
% Euclidean acceleration is equal to the acceleration part of the geodesic
% spray field and the Riemannian acceleration is zero. 
% Reference: "A Riemannian geometry theory of human movement: the geodesic 
% synergy hypothesis", Neilson et al, Human Movement Science 2015.
%
% Parameters:
%   - dq:       joint velocity vector
%   - Ji:       Jacobian until link i for i = 1:nbDOFs at configuration q
%               nbDoFs-dimensional cell 
%   - Mi:       mass-inertia matrix of robot's links 
%               nbDOFs x nbDOFs x nbLinks or nbDOFs x nbDOFs if identical
%               for all links
% 
% Returns:
%   - f2:       Acceleration part of geodesic spray field

% Compute inertia
G = compute_inertia(Ji,Mi);
% Compute inertia derivative w.r.t. joint angles
G_grad = compute_joint_derivative_inertia(Ji,Mi);

% Compute derivatives of kinematic function K = 0.5*v'*I*v
% First partial derivative w.r.t joint angle
dKdc = 0.5*tmprod(tmprod(G_grad,dq',1),dq',2);
% Second partial derivative w.r.t. joint angles and velocity
d2Kdcdv = tmprod(G_grad,dq',2);

% Compute spray acceleration vector f2
% Remark: squeezing dKdc corresponds to computing the contracted
% tensor-vector product.
f2 = G\(squeeze(dKdc) - tmprod(d2Kdcdv,dq',3));

end
