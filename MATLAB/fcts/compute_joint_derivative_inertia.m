function G_grad = compute_joint_derivative_inertia(Ji, Mi)
% Compute the derivative of the inertia matrix with respect to joint angles.
%
% Parameters:
%   - Ji:       Jacobian until link i for i = 1:nbDOFs
%               nbDoFs-dimensional cell 
%   - Mi:       mass-inertia matrix of robot's links 
%               nbDOFs x nbDOFs x nbLinks or nbDOFs x nbDOFs if identical
%               for all links
% 
% Returns:
%   - G_grad:   Gradient of the inertia matrix

nbDOFs = size(Ji,1);
if ismatrix(Mi)
    Mi = repmat(Mi,1,1,nbDOFs);
end

% Derivative of Jacobians with respect to joint angles
dJidq = cell(nbDOFs,1);
for i = 1:nbDOFs
    dJidq{i} = compute_joint_derivative_Jacobian(Ji{i});
end

% Derivative of inertia matrix with respect to joint angles
G_grad = zeros(nbDOFs, nbDOFs, nbDOFs);
for k = 1:nbDOFs
   for i = 1:nbDOFs
       G_grad(:,:,k) = G_grad(:,:,k) + ...
           dJidq{i}(:,:,k)' * Mi(:,:,i) * Ji{i} + Ji{i}' * Mi(:,:,i) * dJidq{i}(:,:,k);
   end
end

% Equivalent computation
% I_grad = zeros(nbDOFs, nbDOFs, nbDOFs);
% for i = 1:nbDOFs
%    I_grad = I_grad + ...
%        tmprod(permute(dJidq{i},[2,1,3]),  Ji{i}' * Mi(:,:,i),2) + tmprod(dJidq{i}, Ji{i}' * Mi(:,:,i),1);
% end

end