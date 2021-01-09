% derive_rolling_dynamics.m
% Zack Woodruff
% 1/8/2021

% This function derives the rolling dynamics equations. It first calculates
% the hand dynamics function for the hand that is directly controlled. It
% then calculates the dynamics for the rolling object. 

% Input:
% param: structure containing important variables, options, and functions

% Output: 
% param: same structure as input, plus dynamic rolling expressions
%   state variables, T matrix functions, fVso, dxddx_hand, K5_, K6_, Ad_Toh_,
%   full_dynamics (if option selected)

function param = derive_rolling_dynamics(param)
disp('Calculating rolling dynamics...')

%% Initialize hand variables
% Positions
syms theta_h_ beta_h_ gamma_h_ real
Phi_sh_ = [theta_h_; beta_h_; gamma_h_]; % Euler angles for {h} frame in {s} frame

syms x_sh_ y_sh_ z_sh_ real;
r_sh_ = [x_sh_; y_sh_; z_sh_]; % position of {h} frame in {s} frame


% Body velocities of hand
syms omega_sh_x_ omega_sh_y_ omega_sh_z_ real 
omega_sh_ = [omega_sh_x_; omega_sh_y_; omega_sh_z_]; % rotational body velocity  in {h} frame

syms v_sh_x_ v_sh_y_ v_sh_z_ real 
v_sh_  = [v_sh_x_; v_sh_y_; v_sh_z_]; % linear body velocity in {h} frame

Vsh_ = [omega_sh_;v_sh_]; % Body twist of the hand 
states_hand_ = [Phi_sh_; r_sh_; omega_sh_; v_sh_]; % full hand state 


% Hand Controls:
% We assume the hand accelerations are directly controlled 
syms alpha_sh_x_ alpha_sh_y_ alpha_sh_z_ a_sh_x_ a_sh_y_ a_sh_z_ real; 
dVsh_ = [alpha_sh_x_; alpha_sh_y_; alpha_sh_z_; a_sh_x_; a_sh_y_; a_sh_z_];


%% Derive Hand Dynamics Equation 
% Hand is directly acceleration controlled 
% States: 
%  [theta_h_, beta_h_, gamma_h_, x_sh_, y_sh_, z_sh_, ...
%   omega_sh_x_, omega_sh_y_, omega_sh_z_,...
%   v_sh_x_, v_sh_y_, v_sh_z_]; 

% Hand dynamics equation
% First solve for Kh matrix that maps between omega and dPhi 
Rsh_ = fEulerToR(Phi_sh_,'XYZ'); % Rotation matrix from xyz euler angles 
Kh_ = KfromR(Rsh_,Phi_sh_); % omega_sh_ = Kh_*dPhi_sh_

dxddx_hand =[inv(Kh_)*omega_sh_;... % Rotational velocities  inv(Kh_) * omega_sh_ = dPhi_sh_
             Rsh_ * v_sh_;... % Linear velocities in {s} frame 
             dVsh_];               % Accelerations from controls 

if param.options.is_simplify
    dxddx_hand = simplify(dxddx_hand); 
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Derive Object Dynamics Equation 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% T matrices
% Between {c_o} and {c_h}
Rpsi_ = param.kinematics.local_geometry.Rpsi_; 
Rchco_ = [Rpsi_, zeros(2,1);...
          0, 0, -1];
Tchco_= RpToTrans(Rchco_,[0;0;0]);

% Between object {o} and {c_o}
Toco_ = simplify(param.kinematics.local_geometry.object.Tici_);
[Roco_, r_oco_] = TransToRp(Toco_);

%  Between hand {h} and {c_h}
Thch_ = simplify(param.kinematics.local_geometry.hand.Tici_);
[Rhch_, r_hch_] = TransToRp(Thch_);

% Between object {o} and {c_h}
Toch_ = simplify(Toco_*TransInv(Tchco_)); 
Tcho_ = TransInv(Toch_);
Rcho_ = Tcho_(1:3,1:3); 

% Between object {o} and {h}
Toh_ = simplify(Toco_ * TransInv(Tchco_) * TransInv(Thch_)); 

% Between {s} and {h}
Tsh_ = RpToTrans(Rsh_,r_sh_);

% R between {o} and {s}
Tho_ = TransInv(Toh_); 
Rho_ = Tho_(1:3,1:3); 
Ros_ = RotInv(Rsh_ * Rho_ );


%% Calculating Eq. (7) from paper
% Extract rolling/pure rolling acceleration constraints from param
friction_model = param.options.friction_model;
Alpha_ = param.variables.Alpha_; 
if strcmp(friction_model,'rolling') % Rolling
    % Rolling includes relative linear acceleration constraint at contact
    % a_roll  from Eq. (41);
    % These were calculated in derive_second_order_kinematics.m
    syms fx_ fy_ fz_ real
    ch_F_contact = [zeros(3,1); fx_; fy_; fz_];
    dVrel = [Alpha_; param.kinematics.a_roll_]; % relative accelerations at the contact
elseif strcmp(friction_model,'pure-rolling') % Pure Rolling
    % Pure-rolling includes relative linear acceleration constraint at contact
    % a_roll  from Eq. (41) and alpha_z_pr from Eq. (45). 
    % These were calculated in derive_second_order_kinematics.m
    syms tau_z_ fx_ fy_ fz_ real
    ch_F_contact = [zeros(2,1); tau_z_; fx_; fy_; fz_];
    dVrel = [Alpha_(1:2); param.kinematics.alpha_z_pr_; param.kinematics.a_roll_]; % relative accelerations at the contact
end

% Adjoint expressions
Ad_Toch_ = Adjoint(Toch_); 
Ad_Tcho_ = Adjoint(Tcho_);
Ad_Toh_ = Adjoint(Toh_); 

Go = param.bodies.object.Go; % Object spatial inertia matrix
omega_rel_ = param.kinematics.omega_rel_fqdq1_; % relative angular velocity at contact omega_rel(q,dq)
Vrel_ = [omega_rel_;0;0;0];  % relative twist at the contact
Vso_ = Ad_Toh_*Vsh_ + Ad_Toch_ * Vrel_; % Body twist of object in {o} from Eq. (2) 
ad_Vso = ad(Vso_);

% Wrench Expressions
gravity = param.dynamics.gravity;
mass_o = param.bodies.object.mass_o;
s_wrench_g_ = [0;0;0;0;0;-gravity*mass_o]; % gravity wrench in {s}
o_wrench_g_ = simplify(blkdiag(Ros_,Ros_)*s_wrench_g_); % gravity wrench in {o}

% Solve for K4
omega_so_ = Vso_(1:3); % body angular velocity in {o}
o_r_opo_ = param.bodies.object.fo_; 
h_r_hph_ = param.bodies.hand.fh_; 
K4_ = ...
Ad_Toh_ * [zeros(3,1); VecToso3(omega_sh_) * (VecToso3(omega_sh_) * h_r_hph_)]...
-Ad_Toch_* [VecToso3(omega_rel_)*(Rcho_*omega_so_) ;zeros(3,1)] ...
-[zeros(3,1); VecToso3(omega_so_) * (VecToso3(omega_so_)*o_r_opo_)]; 

dVsh_; % Hand accelerations 

% Equation 7: LHS = RHS + RHS2 * dVsh_
%LHS_ = Ad_Toch_*dVrel - inv(Go)*Ad_Tcho_'*ch_F_contact;
RHS1_ = inv(Go)*(ad_Vso'*Go*Vso_ + o_wrench_g_) - K4_;
%RHS2_ = - Ad_Toh_;


%% Deriving rolling dynamics from Eq (8) and (9)
P = param.bodies.P;
P_ = param.bodies.P_;
a_ = - inv(Go)*Ad_Tcho_'; 
if strcmp(friction_model,'rolling') % Rolling (Eq 8)
    K5_ =  subs([Ad_Toch_(:,1:3), a_(:,4:6)], P_,P); % from LHS_ of Eq (7)
    K6_ = subs(RHS1_ - Ad_Toch_(:,4:6)*dVrel(4:6) + a_(:,1:3) * ch_F_contact(1:3), P_,P);
elseif strcmp(friction_model,'pure-rolling') % Pure Rolling (Eq 9)
    K5_ = subs([Ad_Toch_(:,1:2), a_(:,3:6)], P_,P); % from LHS_ of Eq (7)
    K6_ = subs(RHS1_ - Ad_Toch_(:,3:6)*dVrel(3:6) + a_(:,1:2) * ch_F_contact(1:2), P_,P); 
end

if param.options.is_simplify
    K5_ = simplify(subs(K5_, P_,P));
    K6_ = simplify(subs(K6_, P_,P));
    Ad_Toh_ = simplify(subs(Ad_Toh_, P_,P));
end


%% Derive full rolling dynamics (Equation 12)
% Solve explicitly for the alpha and wrench terms from equations (8)/(9)
% WARNING: May require a large symbolic matrix inversion, can be avoided
% by settingL 
%   param.options.is_fast_dynamics = true

% States (22): 
%  s = [theta, beta, gamma, ...                 % Phi_sh (1:3) - euler orientation {h} in {s} 
%       r_sh_x, r_sh_y, r_sh_z, ...             % r_sh (4:6) - position of {h} in {s}
%       uo_, vo_, uh_, vh_, psi_,...            % q (7:11) - contact coordinates
%       omega_sh_x, omega_sh_y, omega_sh_z, ... % omega_sh (12:14) - rot body vel in {h}
%       v_sh_x, v_sh_y, v_sh_z, ...             % v_sh (15:17) - linear body vel in {h}
%       duo_, dvo_, duh_, dvh_, dpsi_]          % dq (18:22) - contact coordinate velocities
%
% ds = K7_(s) + K8_(s) * dVsh_
    
if ~param.options.is_fast_dynamics
    % Equation: alpha_lambda = velTerms + accTerms*dVsh_
    invK5_ = inv(K5_);
    velTerms = invK5_* K6_; 
    accTerms = invK5_*(- Ad_Toh_);

    % Contact wrench function (Eq. (10)/(11)) 
    alpha_F_contact_ = velTerms - accTerms * dVsh_; 
    if strcmp(friction_model,'rolling') % Rolling
        F_contact_ = [alpha_F_contact_(4:6)]; % F_contact_roll = [0;0;0;fx;fy;fz] [Eq. (10)]
    elseif strcmp(friction_model,'pure-rolling') % Pure Rolling
        F_contact_ = [alpha_F_contact_(3:6)]; % F_contact_pr = [0;0;tau_z;fx;fy;fz] [Eq. (11)]
    end

    % Deriving equation (12)
    K2_ = subs(param.kinematics.K2_, P_,P);
    K3_ = subs(param.kinematics.K3_, P_,P);

    if strcmp(friction_model,'rolling') % Rolling
        K7_ = [dxddx_hand(1:6);...
               param.variables.dq_;...
               zeros(6,1);...
               K2_ + K3_*[velTerms(1:3); dVrel(4:6)]];
        K8_ = [zeros([6,6]);...
               zeros([5,6]);...
               eye(6);...
               K3_ * [accTerms(1:3,:); zeros([3,6])]]; 
    elseif strcmp(friction_model,'pure-rolling') % Pure Rolling
        K7_ = [dxddx_hand(1:6);...
               param.variables.dq_;...
               zeros(6,1);...
               K2_ + K3_*[velTerms(1:2); dVrel(3:6)]];
        K8_ = [zeros([6,6]);...
               zeros([5,6]);...
               eye(6);...    
               K3_ * [accTerms(1:2,:); zeros([4,6])]];     
    end

    if param.options.is_simplify
        full_dynamics_ = simplify(subs(K7_ + K8_ * dVsh_,P_,P));      
        F_contact_ = simplify(subs(F_contact_,P_,P));   
    end
end


%% Expressions to export
% State and control variables 
param.variables.states_hand_ = states_hand_; % full hand state
param.variables.states_ = [states_hand_(1:6); param.variables.q_; states_hand_(7:12); param.variables.dq_];  % full state s \in 22
param.variables.dVsh_ = dVsh_; % full hand controls

% Transformation matrix functions to use for visualization and analysis
param.functions.fTho = matlabFunction(subs(Tho_,param.bodies.P_,param.bodies.P), 'vars',{param.variables.q_});
param.functions.fThch = matlabFunction(subs(Thch_,param.bodies.P_,param.bodies.P), 'vars',{param.variables.q_});
param.functions.fThco = matlabFunction(subs(Thch_*Tcho_,param.bodies.P_,param.bodies.P), 'vars',{param.variables.q_});
param.functions.fTsh = matlabFunction(Tsh_,'vars',{param.variables.states_(1:6)});

% Relative velocity function
param.functions.fVso = matlabFunction(subs(Vso_,P_,P),'vars',{param.variables.states_});

% full hand dynamics 
param.dynamics.dxddx_hand = dxddx_hand;

% Dynamic rolling equations from Eq. (8)/(9)
param.dynamics.K5_ = K5_;
param.dynamics.K6_ = K6_;
param.dynamics.Ad_Toh_ = Ad_Toh_;

% Full dynamic rolling equations from Eq. (12) 
if ~param.options.is_fast_dynamics
    param.dynamics.full_dynamics_ = full_dynamics_;
    param.dynamics.F_contact_ = F_contact_;
end

disp('    DONE.')


end