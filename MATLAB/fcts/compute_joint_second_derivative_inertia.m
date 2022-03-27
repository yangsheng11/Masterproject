function d2G_dq2 = compute_joint_second_derivative_inertia(Ji, Mi)
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
%   - d2G_dq2:   Second derivative of the inertia matrix with respect to
%                joint angles

nbDOFs = size(Ji,1);
if ismatrix(Mi)
    Mi = repmat(Mi,1,1,nbDOFs);
end

% Derivative of Jacobians with respect to joint angles
dJidq = cell(nbDOFs,1);
for i = 1:nbDOFs
    dJidq{i} = compute_joint_derivative_Jacobian(Ji{i});
end

% Second derivative of Jacobians with respect to joint angles
d2Jidq2 = cell(nbDOFs,1);
for i = 1:nbDOFs
    d2Jidq2{i} = compute_joint_second_derivative_Jacobian(Ji{i});
end

% Second derivative of inertia matrix with respect to joint angles 
% THIS BUGS BECAUSE OF THE TTPROD
% d2I_dq2a = zeros(nbDOFs, nbDOFs, nbDOFs, nbDOFs);
% for i = 1:nbDOFs
%     % dJidq * Mi
%     dJidq_Mi = tmprod(permute(dJidq{i},[2,1,3]), Mi(:,:,i),2);
%     
%     % Term for link i
%     d2I_dq2a = d2I_dq2a + ...
%         tmprod(permute(d2Jidq2{i},[2,1,3,4]),Ji{i}' * Mi(:,:,i),2) + ...
%         tmprod(d2Jidq2{i},Ji{i}' * Mi(:,:,i),1) + ...
%         ttprod(permute(dJidq{i},[2,1,3]),dJidq_Mi,2,2) + ...
%         ttprod(dJidq{i},dJidq_Mi,1,2);
% 
% end

% Second derivative of inertia matrix with respect to joint angles
d2G_dq2 = zeros(nbDOFs, nbDOFs, nbDOFs, nbDOFs);
for m = 1:nbDOFs
    for k = 1:nbDOFs
       for i = 1:nbDOFs
           d2G_dq2(:,:,k,m) = d2G_dq2(:,:,k,m) + ...
               d2Jidq2{i}(:,:,k,m)' * Mi(:,:,i) * Ji{i} + ...
               Ji{i}' * Mi(:,:,i) * d2Jidq2{i}(:,:,k,m) + ...
               dJidq{i}(:,:,k)' * Mi(:,:,i) * dJidq{i}(:,:,m) + ...
               dJidq{i}(:,:,m)' * Mi(:,:,i) * dJidq{i}(:,:,k);
       end
    end
end

end