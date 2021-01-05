function [dx] = f_dynamics_handler(t,x,u,tvec_u,rolling_type_num,is_fast_dynamics)
% f_dynamics_handler 
% This contains the dynamics equations for a hand and an object in rolling
% contact. The hand is directly controlled, the object is subject to the
% rolling constraints assuming rolling is maintained.

% Inputs:
% t = time
% x = [roll_2, pitch_2, yaw_2; ...                       % Phi_2 (1:3)
%      s_p2x; s_p2y; s_p2z; ...                          % p (4:6)
%      u1; v1, u2, v2, psi; ...                          % q (7:11)
%      b_omega_o2_x; b_omega_o2_y; b_omega_o2_z; ...     % omega_b2 (12:14)
%      b_vx_o2; b_vy_o2; b_vz_o2                         % vb2 (15:17)
%      du1; dv1, du2, dv2, dpsi;];                       % dq (18:22)
% u = dVh_ = [alphax_h_; alphay_h_; alphaz_h_; ax_h_; ay_h_; az_h_]
% tvec_u: time array corresponding to controls to use for interpolation
% rolling_type_num: (1) Rolling, (2) Pure Rolling

%Outputs:
% dx = [ ... Velocities : 
%      d/dt[roll_2, pitch_2, yaw_2];...                       % dPhi_2(1:3)
%      s_dp2x; s_dp2y; s_dp2z; ...                            % dp (4:6)
%      du1; dv1, du2, dv2, dpsi; ...                          % dq (7:11)
%      ... Accelerations:
%      b_alpha_o2_x; b_alpha_o2_y; b_alpha_o2_z; ...          % b_alpha_o2 (12:14)
%      b_dvx_o2; b_dvy_o2; b_dvz_o2                           % b_dv_o2 (15:17)
%      ddu1; ddv1, ddu2, ddv2, ddpsi;];                       % ddq (18:22)


%% Extracting object states from state variable x
% Configurations
%Rso2       = reshape(x(1:9),[3,3]); % Object 2 orientation in {s}
%Phiso2     = x(1:3);
%pso2       = x(4:6); % Object 2 position in {s}
q          = x(7:11,:); % contact coordinates

% Velocities
%omega_b2   = x(12:14); % body rotational velocity object 2
%vb2        = x(15:17); % body linear velocity object 2
dq         = x(18:22,:); % contact velocities 


%% Extract controls at current time t
% Feedforward term
N = length(t);  
if N == 1
    b_dV_o2 = interp1(tvec_u,u',t)';
else
    b_dV_o2 = u; % for matrix or constant control input
end


if ~is_fast_dynamics % Fast dynamics in a single equation (Eq. (12))
    dx = autoGen_f_full_dynamics(t,x,b_dV_o2);
else 

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Hand Dynamics
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Matrix form of hand dynamics
    ddHand= autoGen_f_hand_dynamics(t,x([1:6,12:17],:),b_dV_o2);

    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Object Dynamics
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
    % Relative Accelerations
    %rolling_type_num = 1; % (1) Rolling, (2) Pure Rolling
    alphax=zeros(1,N);
    alphay=zeros(1,N); 
    alphaz=zeros(1,N); 
    for i = 1:N 
        K5 = autoGen_f_K5(q(:,i));
        K6 = autoGen_f_K6(x(:,i));
        Ad_Toh_ = autoGen_f_Ad_Toh(q(:,i));
        
        alpha_lambda = inv(K5)*(K6-Ad_Toh_*b_dV_o2(:,i));
        
        %C1val = autoGen_fC1exported_new(q(:,i)); 
        %C2val = autoGen_fC2exported_new(x(:,i), b_dV_o2(:,i)); 
        %alpha_lambda=inv(C1val)*(C2val);
        %alpha_lambda=C1val\(C2val);
        if rolling_type_num == 1 % Rolling
           alphax(i)=alpha_lambda(1); 
           alphay(i)=alpha_lambda(2); 
           alphaz(i)=alpha_lambda(3);
        elseif rolling_type_num == 2 % Pure Rolling    
           alphax(i)=alpha_lambda(1); 
           alphay(i)=alpha_lambda(2);
           alphaz(i) = autoGen_f_alpha_z_pure_rolling([q(:,i);dq(:,i)]);    %alphaz=0;
        end
    end


    % Second Order Kinematics 
    ddq = autoGen_f_second_order_kinematics(t,[q;dq],[alphax;alphay;alphaz]);


    % Extract States 
    dx = zeros(22,N); 
    dx(1:6,:)  = ddHand(1:6,:);
    dx(7:11,:) = dq; 

    % Accelerations into output array 
    dx(12:17,:) = ddHand(7:12,:);
    dx(18:22,:) = ddq; 
end
