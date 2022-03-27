function [df2dc, df2dv] = compute_derivative_acceleration_geodesic_spray_field(dq, Ji, Mi)
% This function computes the derivatives of the acceleration component of 
% the geodesic spray field with respect to the position and velocity.
%
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
%   - df2dc:    derivative of the acceleration part of geodesic spray field
%               with respect to position or current configuration c
%   - df2dv:    derivative of the acceleration part of geodesic spray field
%               with respect to velocity v

% Compute inertia
G = compute_inertia(Ji,Mi);
% Compute inertia derivative with respect to joint angles
G_grad = compute_joint_derivative_inertia(Ji,Mi);
% Compute second inertia derivative with respect to joint angles
G_grad2 = compute_joint_second_derivative_inertia(Ji,Mi);

% Compute derivative of inverse inertia matrix with respect to joint angles
invG = inv(G);
dinvGdq = - tmprod(tmprod(G_grad,invG,1),invG,2);

% Compute derivatives of kinematic function K = 0.5*v'*I*v
% First partial derivative with respect to joint angle
dKdc = 0.5*tmprod(tmprod(G_grad,dq',1),dq',2);
% Second partial derivative with respect to joint angles and velocity
d2Kdcdv = tmprod(G_grad,dq',2);

% Compute derivative of f2 with respect to position 
df2dc = squeeze(tmprod(dinvGdq, (squeeze(dKdc) - tmprod(d2Kdcdv,dq',3))',2)) + ...
    G\(squeeze(0.5*tmprod(tmprod(G_grad2,dq',1),dq',2)) - ...
    squeeze(tmprod(tmprod(G_grad2,dq',2),dq',3)));

% Compute derivative of f2 with respect to velocity 
df2dv = G\(squeeze(0.5*tmprod(G_grad,dq',2))' + ...
    squeeze(0.5*tmprod(G_grad,dq',1))' - ...
    squeeze(tmprod(G_grad,dq',2)) - ...
    tmprod(G_grad,dq',3)' );

% Remark: squeezing a tensor-vector product corresponds to computing the 
% contracted tensor-vector product.

end
