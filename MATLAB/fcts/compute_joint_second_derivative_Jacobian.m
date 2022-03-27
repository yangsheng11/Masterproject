function d2Jdq2 = compute_joint_second_derivative_Jacobian(J)
% Compute the Jacobian second derivative with respect to joint angles 
% (hybrid Jacobian representation).
%
% Parameters:
%   - J:        Jacobian
% 
% Returns:
%   - d2Jdq2:   Second derivative of the Jacobian with respect to joint
%               angles

nb_rows = size(J,1); % task space dim.
nb_cols = size(J,2); % joint space dim.
d2Jdq2 = zeros(nb_rows, nb_cols, nb_cols, nb_cols);
for i = 1:nb_cols
    for j = 1:nb_cols
        for k = 1:nb_cols
            J_i = J(:,i);
            J_j = J(:,j);
            J_k = J(:,k);
            
            if j > i
                if k > i
                    M_dJ_dq = zeros(nb_rows,1);
                else
                    M_dJ_dq(1:3,:) = -cross(J_j(1:3,:) ,cross(J_k(4:6,:),J_i(4:6,:)));
                    M_dJ_dq(4:6,:) = zeros(3,1);
                end
                if k < j
                    dM_dq_J(1:3,:) = -cross(cross(J_k(4:6,:),J_j(1:3,:)), J_i(4:6,:));
                    dM_dq_J(4:6,:) = zeros(3,1);
                else
                    dM_dq_J(1:3,:) = -cross(cross(J_j(4:6,:),J_k(1:3,:)), J_i(4:6,:));
                    dM_dq_J(4:6,:) = zeros(3,1);
                end
                d2Jdq2(:,i,j,k) = dM_dq_J + M_dJ_dq;
            else
                if k > i
                    P_dJ_dq(1:3,:) = -cross(J_j(4:6,:) ,cross(J_k(1:3,:),J_i(4:6,:)));
                    P_dJ_dq(4:6,:) = zeros(3,1);
                else
                    P_dJ_dq(1:3,:) = cross(J_j(4:6,:) ,cross(J_k(4:6,:),J_i(1:3,:)));
                    P_dJ_dq(4:6,:) = cross(J_j(4:6,:) ,cross(J_k(4:6,:),J_i(4:6,:)));
                end
                if k < j
                    dP_dq_J(1:3,:) = cross(cross(J_k(4:6,:),J_j(4:6,:)), J_i(1:3,:));
                    dP_dq_J(4:6,:) = cross(cross(J_k(4:6,:),J_j(4:6,:)), J_i(4:6,:));
                else
                    dP_dq_J = zeros(nb_rows,1);
                end
                d2Jdq2(:,i,j,k) = dP_dq_J + P_dJ_dq;
            end
        end
    end
end

end