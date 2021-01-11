% main_ball_plate_complete_example.m
% Zack Woodruff
% 11/24/2020

% This code derives the rolling dynamics equations for a
% ball (sphere) on a plate (plane) and does an open-loop simulation. 


clear
clc
close all

% Set directory to appropriate directory and add the src folders to the path
tic
current_example_home_directory = fileparts(matlab.desktop.editor.getActiveFilename);
cd(current_example_home_directory);
addpath('../../utilities')
addpath('../../kinematics/')
addpath('../../dynamics')
addpath('../../visualization')
addpath('../../analysis')



%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 1. Initialization 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 1.1 Initialize:
%-1.1.1 Rolling Configuration Variables, 
%-1.1.2 Object Geometry and Inertial
%-1.1.3 Hand Geometry 
%-1.1.4 Combined Parameters
%-1.1.5 Friction Parameters
param = initialize_ball_plate();


%% 1.2 Input Parameters
param.options.is_simplify = true;
param.options.friction_model = 'rolling'; %'pure-rolling' or 'rolling';
param.options.is_fast_dynamics = true;

param.options.is_inclined = true; % Sets whether plate is tilted or horizontal relative to gravity
param.options.export_directory = current_example_home_directory; 
% param.options.model = 'ball-plate';
% param.options.is_generate_figures = false; 



%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 2. Derive Kinematics
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 2.1 Contact Geometry
% 2.2 First-Order Kinematics
% 2.3 Second-Order Kinematics
param  = derive_kinematics(param); 

 

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 3. Dynamics
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 3.1 Derive rolling dynamics
% 3.2 Export dynamics functions
param = derive_export_dyamics(param);



%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 4. Open Loop Simulation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 4.1
% From Section V.C
%Simulate rolling using either the full dynamics or the partial dynamics 

% Time and integration tolerances
param.sim.dt = 0.01;
param.sim.T = 2*pi; 
param.sim.tvec = 0:param.sim.dt:param.sim.T ; % vector of times to export states
param.sim.ode_options = odeset('RelTol',1e-6,'AbsTol',1e-8); % set numerical integration tolerances

% Initial Conditions
if param.options.is_inclined
    param.sim.Xh0 = [0.1; zeros(5,1)]; % Initial hand configuration 
else
    param.sim.Xh0 = [0; zeros(5,1)]; % Initial hand configuration 
end
param.sim.q0 = [pi/2; 0; 0; 0; 0]; % Initial contact coordinates
param.sim.Vsh0 = [0; 0; 7; 0; 0; 0]; % Initial hand body twist
param.sim.omega_rel0 = [1;0; -7]; % Initial relative rotational velocity
        
% Calculate initial dq from omega_rel0: 
param.sim.dq0 = double(subs(param.kinematics.first_order_kinematics_,...
            [param.bodies.P_; param.variables.q_; param.variables.Omega_],...
            [param.bodies.P;  param.sim.q0;       param.sim.omega_rel0]));
param.sim.states0 = [param.sim.Xh0; param.sim.q0; param.sim.Vsh0; param.sim.dq0]; % Initial state vector

% Controls
param.sim.tvec_u = param.sim.tvec; % time vector for control input
param.sim.controls_t = zeros([6, length(param.sim.tvec_u)]);

% Simulate rolling dyanmics
param.sim.states_t = run_dynamic_rolling_simulation(param);



%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 5. Visualization
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 5.1 Visualize rolling trajectory 

% Set Options
param.options.visualization.xlim = [-8,8];
param.options.visualization.ylim = [-7,7];
param.options.visualization.zlim = [-5,4.5];

param.options.visualization.view = [-42,35];
param.options.visualization.figure_size = [7, 7];
param.options.visualization.show_contact = true; 

param.options.visualization.is_export=false; 
param.options.visualization.export_figure_name='plate_ball_spin';

% Run visualiztion
visualize_trajectory(param)



%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 6. Analyze Results 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 6.1
% TODO: 
% - Remove extra scrp code from the end 
analyze_rolling_trajectory(param, param.sim.states_t)


%% 6.2 Compare dynamics rolling results to analytical trajectory solution
% K. Weltner, “Stable circular orbits of freely moving balls on rotating
% discs,” American Journal of Physics, vol. 47, no. 11, pp. 984–986, 1979.

% From Section V.C of the paper
param.sim.analytical_comparison = compare_plate_ball_to_analytical_solution(param,true);


%% 7. End program
toc














