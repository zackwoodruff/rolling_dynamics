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
param.options.is_fast_dynamics = true; 
    
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
% From Section V.A
% Return K5, K6 (K7 and K8 if full derivation) 
param.dynamics.gravity = 9.81; 
param = derive_rolling_dynamics(param);

% TODO: 
% - add full derivation of alpha_z
% -Add status printing to command line 

%% 3.2 Export dynamics functions
% From Section V.B
% Equations used by f_dynamics_handler.m

% TODO:
% -Add status printing to command line 
param.options.export_directory = pwd; 
export_dynamics_functions(param)




%**********************************************
% 4. Open Loop Simulation
%**********************************************
%% 4.1
% From Section V.C
%Simulate using either the full dynamics or the partial dynamics 

% TODO: 
% - Clean up dynamics section (put in new function?)
% - Add handling for full dynamics function to dynamics_handler 

% Simulate Dynamic Rolling
disp('Simulating dynamic rolling...')

% Time and integration tolerances
param.sim.dt = 0.01;
param.sim.T = 2*pi; 
param.sim.tvec = 0:param.sim.dt:param.sim.T ;
param.sim.tvec_u = param.sim.tvec;
param.sim.ode_options = odeset('RelTol',1e-6,'AbsTol',1e-8); % set max step size

% Initial Conditions
param.options.is_inclined = true; 
if param.options.is_inclined
    param.sim.Xh0 = [0.1; zeros(5,1)];
else
    param.sim.Xh0 = [0; zeros(5,1)];
end
param.sim.q0 = [pi/2; 0; 0; 0; 0];
param.sim.Vh0 = [0; 0; 7; 0; 0; 0];
param.sim.omega_xyz0 = [1;0; -7];

param.sim.dq0 = double(subs(param.kinematics.first_order_kinematics_,...
            [param.bodies.P_; param.variables.q_; param.variables.Omega_],...
            [param.bodies.P;  param.sim.q0;       param.sim.omega_xyz0]));
param.sim.states0 = [param.sim.Xh0; param.sim.q0; param.sim.Vh0; param.sim.dq0];

% Controls
param.sim.controls_t = zeros([6,length(param.sim.tvec_u)]);

% Friction type
if strcmp(param.options.friction_model,'rolling')
    param.sim.friction_model_num = 1; % (1) Rolling, 
elseif strcmp(param.options.friction_model,'pure-rolling')
    param.sim.friction_model_num = 2; % (2) Pure Rolling
else
    warning('INVALID FRICTION MODEL TYPE');
end

% Run Simulation
tic
[~,param.sim.states_t] = ode45(@(t,states) f_dynamics_handler(t,states,...
        param.sim.controls_t,...
        param.sim.tvec_u,...
        param.sim.friction_model_num,...
        param.options.is_fast_dynamics),...
        param.sim.tvec,...
        param.sim.states0,...
        param.sim.ode_options);
toc
disp('    DONE: Simulating dynamic rolling.')


%%
%**********************************************
% 5. Visualize Results
%**********************************************
% Set Options
param.options.visualization.xlim = [-8,8];
param.options.visualization.ylim = [-7,7];
param.options.visualization.zlim = [-5,4.5];

param.options.visualization.view = [-42,35];
param.options.visualization.figure_size = [7, 7];
param.options.visualization.show_contact = true; 

param.options.visualization.is_export=false; 
param.options.visualization.export_figure_name='plate_ball_spin';

% Visualize dynamic rolling trajectory  
visualize_trajectory(param)



%%
%**********************************************
% 6. Analyze Results 
%**********************************************
% 6.1
% TODO: 
% - Add code to analyze the output trajectory 
