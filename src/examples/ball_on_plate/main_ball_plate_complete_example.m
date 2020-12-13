% main_ball_plate_complete_example.m
% Zack Woodruff
% 11/24/2020

% This code demonstrates derives the rolling dynamics equations for a
% ball (sphere) on a plate (plane) and does an open-loop simulation. 

% Add the src folders to the path 
tic
addpath(genpath('../../'))

clear
clc
close all


%**********************************************
% 1. Initialization 
%**********************************************
%% 1.1 Input Parameters
%param.options.model = 'ball-plate';
param.options.is_simplify = true;
param.options.is_generate_figures = false; 
param.options.friction_model = 'pure-rolling'; %'pure-rolling' or 'rolling';
    
    
%% 1.2 Initialize:
%-1.2.1 Rolling Configuration Variables, 
%-1.2.2 Object Geometry and Inertial
%-1.2.3 Hand Geometry 
param = initialize_ball_plate(param);


    
%**********************************************
% 2. Derive Kinematics
%**********************************************
%% 2.1 Contact Geometry
% From Appendix B-A

% Object Geometry:
%   param.geo.geometry_o = diffgeo2(param.geo.fo_,param.geo.Uo_);
param.kinematics.local_geometry.object = ...
    derive_local_contact_geometry_expressions(param.bodies.object.fo_,...
                                              param.variables.Uo_);

% Hand Geometry:
param.kinematics.local_geometry.hand = ...
    derive_local_contact_geometry_expressions(param.bodies.hand.fh_,...
                                              param.variables.Uh_);


%% 2.2 First Order Kinematics
% From Appendix B-B
%**Return K1 and K1*Omega_ and relative velocity from dq expressions
% and functions
param = derive_first_order_kinematics(param);


%% 2.3 Second Order Kinematics
% From Appendix B-C
% Return: K2_, K3_, Axyz_rolling_, alpha_z_pure_rolling_, second order kinematics_ , K4_, 
 param = derive_second_order_kinematics(param);

 
%% 
%**********************************************
% 3. Derive Dynamics
%**********************************************
%% 3.1
% Return K5, K6 (K7 and K8 if full derivation) 
param.dynamics.gravity = 9.81; 
param = derive_rolling_dynamics(param);



%% Export dynamics functions
directory = pwd; % Export functions to the home directory 
is_optimize = param.options.is_simplify; % Should we optimize the exported functions? 
    
param.options.is_fast_dynamics = true; 
if param.options.is_fast_dynamics
    % Export functions for fast dynamics
    % This method avoids a large symbolic matrix inversion
    
    syms t_ real
    q_ = param.variables.q_;
    qdq_ = [q_; param.variables.dq_];
    states_hand_ = param.variables.states_hand_; 
    states_ = [states_hand_(1:6); param.variables.q_; states_hand_(7:12); param.variables.dq_];
    controls_ = param.variables.dVh_; 
        
    % Second Order Kinematics
    matlabFunction(subs(param.kinematics.second_order_kinematics_,param.bodies.P_,param.bodies.P),...
        'File',[directory '\autoGen_f_second_order_kinematics'],'Optimize',is_optimize,...
        'Outputs', {'dYdt'}, 'Vars',{t_,qdq_,param.variables.Alpha_});
    
% Pure Rolling Constraint
    matlabFunction(param.kinematics.alpha_z_,'File',[directory '\autoGen_f_alpha_z_pure_rolling'],...
    'Optimize',is_optimize,'Vars',{qdq_});
    
    % Hand Dynamics
    matlabFunction(param.dynamics.dxddx_hand,'file',[directory, '\autoGen_f_hand_dynamics'],...
        'Optimize',is_optimize,'Vars',{t_, states_hand_, controls_});
    
    % Object Dynamics
    matlabFunction(param.dynamics.K5_,'File',[directory '\autoGen_f_K5'],...
        'Optimize',is_optimize,'Vars',{q_});
    matlabFunction(param.dynamics.K6_,'File',[directory '\autoGen_f_K6'],...
        'Optimize',is_optimize,'Vars',{states_});
    matlabFunction(param.dynamics.Ad_Toh_,'File',[directory '\autoGen_f_Ad_Toh'],...
        'Optimize',is_optimize,'Vars',{q_});
    
else
    warning('ENTER FULL DYNAMICS DERIVATION HERE')
end



%**********************************************
% 4. Open Loop Simulation
%**********************************************
%% 4.1
%Simulate using either the full dynamics or the partial dynamics 

% Simulate Dynamic Rolling
disp('Simulating dynamic rolling...')
param.options.ode_options = odeset('RelTol',1e-6,'AbsTol',1e-8); % set max step size
%param.options.ode_options = odeset('RelTol',1e-12,'AbsTol',1e-12); % set max step size


% Initial Conditions
param.options.is_inclined = false; 
if param.options.is_inclined
    param.mpp.Xh0 = [0.1; zeros(5,1)];
else
    param.mpp.Xh0 = [0; zeros(5,1)];
end
param.mpp.q0 = [pi/2; 0; 0; 0; 0];

param.mpp.Vh0 = [0; 0; 7; 0; 0; 0];
omega_xyz0 = [1;0; -7];
param.mpp.dq0 = double(subs(param.kinematics.first_order_kinematics_, [param.bodies.P_;param.variables.q_;param.variables.Omega_],...
                              [param.bodies.P;param.mpp.q0;omega_xyz0]));


param.mpp.states0 = [param.mpp.Xh0; param.mpp.q0; param.mpp.Vh0; param.mpp.dq0];

disp('Simulating dynamic rolling...')
if strcmp(param.options.friction_model,'rolling')
    rolling_type_num = 1; % (1) Rolling, (2) Pure Rolling
elseif strcmp(param.options.friction_model,'pure-rolling')
    rolling_type_num = 2;
else
    warning('INVALID FRICTION MODEL TYPE');
end
options = param.options.ode_options;
dt = 0.01; 
tvec = 0:dt:5;
tvec_u = tvec;
controls_t = zeros([6,length(tvec_u)]);
states0 = param.mpp.states0;
tic
[~,states] = ode45(@(t,states) f_dynamics_handler(t,states,controls_t,tvec_u,rolling_type_num), tvec, states0, options);
toc
disp('    DONE: Simulating dynamic rolling.')


%**********************************************
% 5. Visualize Results
%**********************************************
%% 5.1 



%**********************************************
% 6. Analyze Results 
%**********************************************
%% 6.1

