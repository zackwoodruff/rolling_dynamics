% main_ball_plate_complete_example.m
% Zack Woodruff
% 11/24/2020

% This code demonstrates derives the rolling dynamics equations for a
% ball (sphere) on a plate (plane) and does an open-loop simulation. 

% Add the src folders to the path 
addpath(genpath('../../'))

clear
clc
close all


%**********************************************
% 1. Initialization 
%**********************************************
%% 1.1 Input Parameters
    %param.options.model = 'ball-plate';
    param.options.is_simplify = false;
    param.options.is_generate_figures = false; 
    param.options.friction_model = 'pure-rolling'; %'pure-rolling' or 'rolling';
    
    
%% 1.2 Initialize:
    %       -1.2.1 Rolling Configuration Variables, 
    %       -1.2.2 Object Geometry and Inertial
    %       -1.2.3 Hand Geometry 
    param = initialize_ball_plate(param);


    
%**********************************************
% 2. Derive Kinematics
%**********************************************
%% 2.1 Contact Geometry
% From Appendix B-A
clc

% Object Geometry:
%   param.geo.geometry_o = diffgeo2(param.geo.fo_,param.geo.Uo_);
param.kinematics.local_geometry.object = derive_local_contact_geometry_expressions(param.bodies.object.fo_,param.variables.Uo_);
param.kinematics.local_geometry.object

% Hand Geometry:
%   param.geo.geometry_h = diffgeo2(param.geo.fh_,param.geo.Uh_);
param.kinematics.local_geometry.hand = derive_local_contact_geometry_expressions(param.bodies.hand.fh_,param.variables.Uh_);
param.kinematics.local_geometry.hand


% param.kinematics.geometry = 
% Get rid of this? 
%geometry_oh = parse_object_geometries(param.geo.geometry_o,param.geo.geometry_h);


%% 2.2 First Order Kinematics
%**Return K1 and K1*Omega_ and relative velocity from dq expressions
% and functions 

% From Appendix B-B
disp('Calculating First-Order Kinematics...')

% Return: K1
param.kinematics.local_geometry.Rpsi_ = [cos(param.variables.q_(5)),-sin(param.variables.q_(5));-sin(param.variables.q_(5)),-cos(param.variables.q_(5))]; 
param.kinematics.local_geometry.E1 = [0,-1;1,0];
param.kinematics.local_geometry.object.H_tilda = param.kinematics.local_geometry.Rpsi_ * param.kinematics.local_geometry.object.H_ * param.kinematics.local_geometry.Rpsi_;

% Initializing velocity expressions
syms omegax_ omegay_ omegaz_ real
param.variables.omegaxy_ = [omegax_;omegay_];
param.variables.Omega_ = [omegax_; omegay_; omegaz_];

% make new function, extract these from param
    sqrtGo = param.kinematics.local_geometry.object.sqrtG_; 
    Rpsi = param.kinematics.local_geometry.Rpsi_;
    Ho_tilda = param.kinematics.local_geometry.object.H_tilda; 
    Hh = param.kinematics.local_geometry.hand.H_; 

    sqrtGh = param.kinematics.local_geometry.hand.sqrtG_;

    sigma_h = param.kinematics.local_geometry.hand.sigma_;
    Gamma_h = param.kinematics.local_geometry.hand.Gamma_;

    sigma_o = param.kinematics.local_geometry.object.sigma_;
    Gamma_o = param.kinematics.local_geometry.object.Gamma_;
    E1 = param.kinematics.local_geometry.E1; 
    
    
    K1o = inv(sqrtGo)*Rpsi*inv(Ho_tilda + Hh)*E1;
    K1h = inv(sqrtGh)*inv(Ho_tilda + Hh)*E1;
    K1_psi = sigma_o*Gamma_o*K1o + sigma_h*Gamma_h*K1h;
    K1 = [K1o,    zeros(2,1);...
          K1h,    zeros(2,1);...
          K1_psi, -1];
    if param.options.is_simplify
        K1 = simplify(K1);
    end
    
    % MAYBE? Expressions for first order rolling constraints on dq_
    % rotational velocities in terms of contact coordinates
    dUo_ = param.variables.dq_(1:2);
    dUh_ = param.variables.dq_(3:4);
    dpsi_ = param.variables.dq_(5);
    omega_xy1 = E1\(Ho_tilda+Hh)*Rpsi'*sqrtGo*dUo_;
    omega_xy2 = E1\(Ho_tilda+Hh)*sqrtGh*dUh_;
    omegaz = sigma_o*Gamma_o*dUo_+sigma_h*Gamma_h*dUh_-dpsi_;
    omega_z1 = omegaz;
    omega_z2 = omegaz;
    
    %omega_xy1= (inv(K1o)*param.geo.dq_(1:2));
    %omega_z1 = (K1_psi*omega_xy1 - param.geo.dq_(5));
    
    %omega_xy2= (inv(K1h)*param.geo.dq_(3:4));
    %omega_z2 = (K1_psi*omega_xy2 - param.geo.dq_(5));
    
    omega_rel_q1 =[omega_xy1; omega_z1]; 
    omega_rel_q2 =[omega_xy2; omega_z2];
    
    if param.options.is_simplify
        omega_rel_q1 = simplify(omega_rel_q1);
        omega_rel_q2 = simplify(omega_rel_q2);
    end
      
%**Return K1 and K1*Omega_ and relative velocity from dq expressions
    %and functions 
    param.K1=K1;
    param.kinematics1 = K1*param.variables.Omega_; 
    
disp('    DONE: Calculating First-Order Kinematics.')    
%%


%**Return K1 and K1*Omega_ and relative velocity from dq expressions
    %and functions 
    param.K1=K1;
    param.kinematics1 = K1*Omega_; 
    
    % MAYBE? Expressions for first order rolling constraints on dq_
    % rotational velocities in terms of contact coordinates
    dUo_ = param.geo.dq_(1:2);
    dUh_ = param.geo.dq_(3:4);
    dpsi_ = param.geo.dq_(5);
    omega_xy1 = E1\(Ho_tilda+Hh)*Rpsi'*sqrtGo*dUo_;
    omega_xy2 = E1\(Ho_tilda+Hh)*sqrtGh*dUh_;
    omegaz = sigma_o*Gamma_o*dUo_+sigma_h*Gamma_h*dUh_-dpsi_;
    omega_z1 = omegaz;
    omega_z2 = omegaz;
    2
    %omega_xy1= (inv(K1o)*param.geo.dq_(1:2));
    %omega_z1 = (K1_psi*omega_xy1 - param.geo.dq_(5));
    3
    %omega_xy2= (inv(K1h)*param.geo.dq_(3:4));
    %omega_z2 = (K1_psi*omega_xy2 - param.geo.dq_(5));
    4
    omega_rel_q1 =[omega_xy1; omega_z1]; 
    omega_rel_q2 =[omega_xy2; omega_z2];
    
    if param.options.is_simplify
        omega_rel_q1 = simplify(omega_rel_q1);
        omega_rel_q2 = simplify(omega_rel_q2);
    end
    disp('    DONE: Calculating First-Order Kinematics.')












%% 2.3 Second Order Kinematics
% Return: K2, K3, K4



%**********************************************
% 3. Derive Dynamics
%**********************************************
%% 3.1
% Return K5, K6 (K7 and K8 if full derivation) 
%param.dynamics.gravity = 9.81; 



%**********************************************
% 4. Open Loop Simulation
%**********************************************
%% 4.1
%Simulate using either the full dynamics or the partial dynamics 



%**********************************************
% 5. Visualize Results
%**********************************************
%% 5.1 



%**********************************************
% 6. Analyze Results 
%**********************************************
%% 6.1

