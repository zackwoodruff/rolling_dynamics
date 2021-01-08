% derive_first_order_kinematics.m
% Zack Woodruff
% 1/8/2021

% Input:
% param: structure containing important variables, options, and functions

% Output: 
% param: same structure as input, plus first order kinematics expressions
%          Omega_, K1_, first_order_kinematics_, omega_rel_fqdq1_
%          Rpsi_, E1, Ho_tilda_

function param = derive_first_order_kinematics(param) 
% From Appendix B-B
disp('Calculating first-order kinematics...')


%% Initializing symbolic velocity expressions
syms omega_x_ omega_y_ omega_z_ real
omega_xy_ = [omega_x_;omega_y_];
Omega_ = [omega_x_; omega_y_; omega_z_];


%% Derive K1 from Eq. (31)
% dq = K1(q)*omega_rel

% Defining terms from Appendix B-B
E1 = [0,-1;1,0]; 
psi_ = param.variables.q_(5);
Rpsi_ =  [cos(psi_),-sin(psi_);...
         -sin(psi_),-cos(psi_)];
Ho_ = param.kinematics.local_geometry.object.H_;
Ho_tilda_ = Rpsi_ * Ho_ * Rpsi_;


% Extracting differental geometry expressions from param
Hh_ = param.kinematics.local_geometry.hand.H_; 

sqrtGo_ = param.kinematics.local_geometry.object.sqrtG_; 
sqrtGh_ = param.kinematics.local_geometry.hand.sqrtG_;

sigma_h_ = param.kinematics.local_geometry.hand.sigma_;
Gamma_h_ = param.kinematics.local_geometry.hand.Gamma_;

sigma_o_ = param.kinematics.local_geometry.object.sigma_;
Gamma_o_ = param.kinematics.local_geometry.object.Gamma_;


% Solving for Eq. (31)
K1o_ = inv(sqrtGo_)*Rpsi_*inv(Ho_tilda_ + Hh_)*E1;
K1h_ = inv(sqrtGh_)*inv(Ho_tilda_ + Hh_)*E1;
K1_psi_ = sigma_o_*Gamma_o_*K1o_ + sigma_h_*Gamma_h_*K1h_;
K1_ = [K1o_,    zeros(2,1);...
      K1h_,    zeros(2,1);...
      K1_psi_, -1];
if param.options.is_simplify
    K1_ = simplify(K1_);
end
    

%% Derive relative velocities omega_rel in terms of q and dq
% Inverting the equation for K1 to find omega_xyz in terms of q, dq
% There are five equations for dq, and three for omega_xyz so there are
% multiple expressions 

% symbolic contact coordinate velocity variables 
dUo_ = param.variables.dq_(1:2);
dUh_ = param.variables.dq_(3:4);
dpsi_ = param.variables.dq_(5);

% omega_xyz as f(q,dq) using equations, 1, 2, and 5; 
omega_xy_o_ = E1\(Ho_tilda_+Hh_)*Rpsi_'*sqrtGo_*dUo_;
omega_z_qdq_ = sigma_o_*Gamma_o_*dUo_+sigma_h_*Gamma_h_*dUh_-dpsi_;
omega_rel_fqdq1_ =[omega_xy_o_; omega_z_qdq_]; 

% omega_xyz as f(q,dq) using equations, 3, 4, and 5; 
omega_xy2_ = E1\(Ho_tilda_+Hh_)*sqrtGh_*dUh_;
omega_rel_fqdq2_ =[omega_xy2_; omega_z_qdq_];

if param.options.is_simplify
    omega_rel_fqdq1_ = simplify(omega_rel_fqdq1_);
    omega_rel_fqdq2_ = simplify(omega_rel_fqdq2_);
end


%% TODO: Expressions for first order rolling constraints on dq_?


%% Expressions to export
% Symbolic velocity variables. 
param.variables.omega_xy_ = omega_xy_;
param.variables.Omega_ = Omega_;

% Additional terms from Appendix B-B
param.kinematics.local_geometry.Rpsi_ =  Rpsi_;
param.kinematics.local_geometry.E1 = E1;
param.kinematics.local_geometry.object.Ho_tilda_ = Ho_tilda_; 

% First order kinematics expressions 
param.kinematics.K1_=K1_; % dq = K1(q)*omega_rel
param.kinematics.first_order_kinematics_ = K1_*param.variables.Omega_;  % full first order kinematics expression 

% Relative contact velocities omega_rel as f(q,dq)
param.kinematics.omega_rel_fqdq1_ = omega_rel_fqdq1_;
param.kinematics.omega_rel_fqdq2_ = omega_rel_fqdq2_;

disp('    DONE.')   
end




