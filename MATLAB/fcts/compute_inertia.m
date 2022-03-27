function G = compute_inertia(Ji,Mi)
% Compute the inertia matrix.
% Based on :
% - "A Lie group formulation of robot dynamics" (F. C. Park, J. E. Bobrow, 
% S. R. Ploen, equations 41-46
% - "A Mathematical Introduction to Robotic Manipulation" (R. M. Murray,
% Z. Li, S. Shankar Sastry), chapter 4, equation 4.19
%
% Parameters:
%   - Ji:       Jacobian until link i for i = 1:nbDOFs
%               nbDoFs-dimensional cell 
%   - Mi:       mass-inertia matrix of robot's links 
%               nbDOFs x nbDOFs x nbLinks or nbDOFs x nbDOFs if identical
%               for all links
% 
% Returns:
%   - G:        Mass-inertia matrix of the robot

nbDOFs = size(Ji,1);
if ismatrix(Mi)
    Mi = repmat(Mi,1,1,nbDOFs);
end

G = zeros(nbDOFs);
for i = 1:nbDOFs
   G = G + Ji{i}'*Mi(:,:,i)*Ji{i};
end

end