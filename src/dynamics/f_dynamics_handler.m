% f_dynamics_handler.m
% Zack Woodruff
% 1/8/2021

% f_dynamics_handler 
% This contains the dynamics equations for a hand and an object in rolling
% contact. The hand is directly controlled, the object is subject to the
% rolling constraints assuming rolling is maintained.

% Inputs:
% t = time in second 
% x = [theta_h, beta_h, gamma_h, ...            % Phi_sh (1:3) - euler orientation {h} in {s} 
%      r_sh_x, r_sh_y, r_sh_z, ...              % r_sh (4:6) - position of {h} in {s}
%      uo_, vo_, uh_, vh_, psi_,...             % q (7:11) - contact coordinates
%      omega_sh_x, omega_sh_y, omega_sh_z, ...  % omega_sh (12:14) - rot body vel {h}
%      v_sh_x, v_sh_y, v_sh_z, ...              % v_sh (15:17) - linear body vel {h}
%      duo_, dvo_, duh_, dvh_, dpsi_]           % dq (18:22) - contact coordinate velocities
% u = dVsh_ = [alpha_sh_x_; alpha_sh_y_; alpha_sh_z_; a_sh_x_; a_sh_y_; a_sh_z_]
% tvec_u: time array corresponding to controls to use for interpolation
% rolling_type_num: (1) rolling, (2) pure-rolling
% is_fast_dynamics: chooses between full and partial dynamics equations

% Outputs:
% dx = [d/dt[theta_h, beta_h, gamma_h], ...      % dPhi_sh (1:3) - euler velocities {h} in {s} 
%       v_sh_x, v_sh_y, v_sh_z, ...              % v_sh (4:6) - linear velocity of {h} in {s}
%       duo_, dvo_, duh_, dvh_, dpsi_,...        % q (7:11) - contact coordinate velocities
%       alpha_sh_x, alpha_sh_y, alpha_sh_z, ...  % alpha_sh (12:14) - rot body accelerations {h}
%       a_sh_x, a_sh_y, a_sh_z, ...              % a_sh (15:17) - linear body accelerations {h}
%       dduo_, ddvo_, dduh_, ddvh_, ddpsi_]      % ddq (18:22) - contact coordinate accelerations 


function dx = f_dynamics_handler(t,x,u,tvec_u,rolling_type_num,is_fast_dynamics)

%% Extracting object states from state variable x
% contact configurations
q = x(7:11,:); % contact coordinates

% contact velocities
dq = x(18:22,:); % contact velocities 


%% Extract controls at current time t
% Feedforward term
N = length(t);  
if N == 1
    dVsh = interp1(tvec_u,u',t)';
else
    dVsh = u; % for matrix or constant control input
end


%%  Calculate dx 
if ~is_fast_dynamics % Dynamics in a single equation (Eq. (12))
    dx = autoGen_f_full_dynamics(t,x,dVsh);
else 

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Hand Dynamics
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Matrix form of hand dynamics
    ddHand= autoGen_f_hand_dynamics(t,x([1:6,12:17],:),dVsh);

    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Object Dynamics
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
    % Relative accelerations from Eq. (8)/(9) 
    %rolling_type_num = 1; % (1) Rolling, (2) Pure Rolling
    alphax=zeros(1,N);
    alphay=zeros(1,N); 
    alphaz=zeros(1,N); 
    for i = 1:N 
        K5 = autoGen_f_K5(q(:,i));
        K6 = autoGen_f_K6(x(:,i));
        Ad_Toh_ = autoGen_f_Ad_Toh(q(:,i));
        
        alpha_lambda = inv(K5)*(K6-Ad_Toh_*dVsh(:,i)); % includes relative acceleration and contact wrench terms 
        if rolling_type_num == 1 % Rolling
           alphax(i)=alpha_lambda(1); 
           alphay(i)=alpha_lambda(2); 
           alphaz(i)=alpha_lambda(3);
        elseif rolling_type_num == 2 % Pure Rolling    
           alphax(i)=alpha_lambda(1); 
           alphay(i)=alpha_lambda(2);
           alphaz(i) = autoGen_f_alpha_z_pure_rolling([q(:,i);dq(:,i)]);
        end
    end

    % Solve fore ddq using the second-order kinematics 
    ddq = autoGen_f_second_order_kinematics(t,[q;dq],[alphax;alphay;alphaz]);

    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Return full dynamics
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
    dx = zeros(22,N); 
    dx(1:6,:)  = ddHand(1:6,:);   % hand velocities 
    dx(7:11,:) = dq;              % contact coordinate velocities 
    dx(12:17,:) = ddHand(7:12,:); % hand accelerations
    dx(18:22,:) = ddq;            % contact coordinate accelerations 
end
