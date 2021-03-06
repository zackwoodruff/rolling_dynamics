% derive_second_order_kinematics.m
% Zack Woodruff
% 1/8/2021

% Input:
% param: structure containing important variables, options, and functions

% Output: 
% param: same structure as input, plus second-order kinematics expressions
%        (Alpha_, K2_, K3_, second_order_kinematics_, a_roll_, alpha_z_pr_)

function param = derive_second_order_kinematics(param)
disp('Calculating second-order kinematics...')
    
%% Extract surface geometry terms from param
Rpsi_ = param.kinematics.local_geometry.Rpsi_;
E1 = param.kinematics.local_geometry.E1; 

Ho_ = param.kinematics.local_geometry.object.H_;
sqrtGo_ = param.kinematics.local_geometry.object.sqrtG_;

sqrtGh_ = param.kinematics.local_geometry.hand.sqrtG_;
Hh_ = param.kinematics.local_geometry.hand.H_; 

sigma_h_ = param.kinematics.local_geometry.hand.sigma_;
Gamma_h_ = param.kinematics.local_geometry.hand.Gamma_;
Gamma_h_bar_ = param.kinematics.local_geometry.hand.Gammabar_;
Gamma_h_barbar_ = param.kinematics.local_geometry.hand.Gammabarbar_;
Lh_bar_ = param.kinematics.local_geometry.hand.Lbar_;
Lh_barbar_ = param.kinematics.local_geometry.hand.Lbarbar_;

sigma_o_ = param.kinematics.local_geometry.object.sigma_;
Gamma_o_ = param.kinematics.local_geometry.object.Gamma_;
Gamma_o_bar_ = param.kinematics.local_geometry.object.Gammabar_;
Gamma_o_barbar_ = param.kinematics.local_geometry.object.Gammabarbar_;
Lo_ = param.kinematics.local_geometry.object.L_;
Lo_bar_ = param.kinematics.local_geometry.object.Lbar_;
Lo_barbar_ = param.kinematics.local_geometry.object.Lbarbar_;

P = param.bodies.P; 
P_ = param.bodies.P_;
%% Extract velocity terms from param
dq_=param.variables.dq_;
dUo_=dq_(1:2);
dUh_=dq_(3:4);
dpsi_=dq_(5);

duo_=dUo_(1);
dvo_=dUo_(2);
duh_=dUh_(1);
dvh_=dUh_(2);
Wo_ = [duo_^2; duo_*dvo_; dvo_^2];
Wh_ = [duh_^2; duh_*dvh_; dvh_^2];

% omega_rel as f(q,dq) 
omega_xy_ = param.kinematics.omega_rel_fqdq1_(1:2); 
omega_z_ = param.kinematics.omega_rel_fqdq1_(3); 

    
%% Calculate K2 (velocity terms) Eq. (39) 
% ddq = K2(q,omega_rel) + K3(q)*dVrel

%K2a
pt1_ = inv([Rpsi_*sqrtGo_   , -sqrtGh_;...
           Rpsi_*E1*Ho_*sqrtGo_, -E1*Hh_*sqrtGh_]);   
pt2_ = [-Rpsi_*sqrtGo_*Gamma_o_bar_;...
        Rpsi_*E1*inv(sqrtGo_)*Lo_barbar_]*Wo_;
pt3_ = [sqrtGh_*Gamma_h_bar_;...
       -E1*inv(sqrtGh_)*Lh_barbar_]*Wh_;  
pt4_ = [-2*omega_z_*E1*Rpsi_*sqrtGo_, zeros(2,2);...
       -omega_z_*Rpsi_*Ho_*sqrtGo_, -dpsi_*Hh_*sqrtGh_]*[dUo_;dUh_];
pt5_ = [zeros(2,1); sigma_o_*Gamma_o_*dUo_*E1*omega_xy_];

K2a_ = pt1_ *(pt2_+pt3_+pt4_+pt5_); 


%K2b
K2b1_ = (E1*omega_xy_).'*Rpsi_*E1*inv(sqrtGo_)*Lo_*dUo_;
K2b_ = K2b1_...
    +sigma_o_*Gamma_o_barbar_*Wo_...
    +sigma_h_*Gamma_h_barbar_*Wh_... 
    + [sigma_o_*Gamma_o_, sigma_h_*Gamma_h_] * K2a_;

% K2 - Eq. (39)
K2_=[K2a_; K2b_];  
if param.options.is_simplify
    K2_ = simplify(K2_); 
end


%% Calculate K3 (position and acceleraiton terms) Eq. (40)
% ddq = K2(q,omega_rel) + K3(q)*dVrel

E2 = [0, 0, 0, -1, 0 ,0;...
      0, 0, 0, 0, -1, 0;...
      eye(3), zeros(3,3)];

K3a_ = [pt1_, zeros(4,1);...
       zeros(1,4), 1]*E2; 

%K3 - Eq. (40)
K3_= [eye(4), zeros(4,1);...
     sigma_o_*Gamma_o_, sigma_h_*Gamma_h_, -1] * K3a_; 
if param.options.is_simplify
    K3_ = simplify(K3_);
end


%% Second Order Rolling Constraint a_roll
% Linear acceleration constraints - Eq. (41)
axy_ = -E1*Rpsi_*sqrtGo_*dUo_*omega_z_;
az_ = Lo_bar_*Wo_+Lh_bar_*Wh_+2*(E1'*omega_xy_)'*Rpsi_*sqrtGo_*dUo_; % Non-breaking contact constraint 
a_roll_ = [axy_; az_]; % Eq. (41) 

if param.options.is_simplify
    a_roll_ = simplify(subs(a_roll_,P_,P)); 
else
    a_roll_ = subs(a_roll_,P_,P); 
end


%% Second Pure-Rolling Constraint alpha_z_pr
%Full derivation of pure rolling expression 

% Initialize variables for second order kinematics equations
syms a_x_ a_y_ real; 
syms alpha_x_ alpha_y_ alpha_z_ real
Alpha_ = [alpha_x_; alpha_y_; alpha_z_];

%pt6_ = [zeros(2,1); Alpha_(1:2)];
pt7_ = [a_x_; a_y_; zeros(2,1)];

C5=[sigma_o_*Gamma_o_, sigma_h_*Gamma_h_, -1];

q_=param.variables.q_;
dC5 = fdiff_t_syms(C5,q_,dq_);

f6_ = dC5*dq_; 
f7_ = C5(1:4);
%ddPsi_purerolling_=f6_ + f7_*param.geo.ddU_(1:4);

% ddU1U2 = f8+f9*[v.alpha.x; v.alpha.y]
f8_=pt1_*(pt2_ + pt3_ + pt4_ + pt5_ - pt7_);
f9_=pt1_(:,3:4);

% ddpsi = f10 +f11*ddU_12 - alphaz
f10_ = sigma_o_*Gamma_o_barbar_*Wo_+ sigma_h_*Gamma_h_barbar_*Wh_+ K2b1_;
f11_ = [sigma_o_*Gamma_o_, sigma_h_*Gamma_h_];

% alpha_z_pure_rolling = f12_ + f13_*[alpha_x; alpha_y]; 
f12_=f10_-f6_+(f11_-f7_)*f8_;
f13_=(f11_-f7_)*f9_;

% Check consistency
alpha_z_pure_rolling = f12_ + f13_*[Alpha_(1:2)];
if param.options.is_simplify
    alpha_z_pure_rolling = simplify(alpha_z_pure_rolling);
end


%% Compare results to EQ.(45) in paper

% Rotational acceleration constraints (pure-rolling) - Eq. 45)
alpha_z_pr_ = ((E1*omega_xy_).'*Rpsi_*E1*inv(sqrtGo_)*Lo_*dUo_);

temp = simplify(alpha_z_pure_rolling-alpha_z_pr_); 
if  temp~=0
    temp
    error('alpha_z_pr assumption may be invalid')
end

% Simplify the reults
if param.options.is_simplify
    alpha_z_pr_ = simplify(subs(alpha_z_pr_,P_,P));
else
    alpha_z_pr_ = subs(alpha_z_pr_,P_,P);
end


%% Full second-order kinematics equation with linear acceleration constraints a_roll substituded
second_order_kinematics_ = (K2_+K3_*[Alpha_; a_roll_]); 
if param.options.is_simplify
    second_order_kinematics_ = simplify(second_order_kinematics_); 
end


%% Expressions to export
% Symbolic acceleration variables 
param.variables.Alpha_ = Alpha_;

% Partial second order kinematics terms
param.kinematics.K2_ = K2_;
param.kinematics.K3_ = K3_;

% Full second order kinematics term
param.kinematics.second_order_kinematics_ = second_order_kinematics_;

% Velocity and acceleration terms 
param.kinematics.a_roll_ = a_roll_;
param.kinematics.alpha_z_pr_ = alpha_z_pr_;

disp('    DONE.')
end





