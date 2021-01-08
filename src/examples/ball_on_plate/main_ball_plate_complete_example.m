% main_ball_plate_complete_example.m
% Zack Woodruff
% 11/24/2020

% This code demonstrates derives the rolling dynamics equations for a
% ball (sphere) on a plate (plane) and does an open-loop simulation. 

% Add the src folders to the path 
tic
addpath(genpath('../../'))
set(0,'DefaultFigureWindowStyle','docked')

clear
clc
close all



%**********************************************
% 1. Initialization 
%**********************************************
%% 1.1 Initialize:
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
% param.options.model = 'ball-plate';
% param.options.is_generate_figures = false; 


%**********************************************
% 2. Derive Kinematics
%**********************************************
%% 2.1 Contact Geometry
% From Appendix B
disp('Calculating symbolic local contact geometry expressions...')

%-2.1.1 Object Geometry:
param.kinematics.local_geometry.object = ...
    derive_local_contact_geometry_expressions(param.bodies.object.fo_,...
                                              param.variables.Uo_);
%-2.1.2 Hand Geometry:
param.kinematics.local_geometry.hand = ...
    derive_local_contact_geometry_expressions(param.bodies.hand.fh_,...
                                              param.variables.Uh_);
                                          
disp('    DONE.');


%% 2.2 First Order Kinematics
% From Appendix B-B
% Returns: Omega_, K1_, first_order_kinematics_, omega_rel_fqdq1_
%          Rpsi_, E1, Ho_tilda_
% TODO:
% - include first order velocity constraints? 
param = derive_first_order_kinematics(param);


%% 2.3 Second Order Kinematics
% From Appendix B-C
% Returns: Alpha_, K2_, K3_, second_order_kinematics_, a_roll_, alpha_z_pr_
 param = derive_second_order_kinematics(param);

 
 
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


%% 3.2 Export dynamics functions
% From Section V.B
% Equations used by f_dynamics_handler.m
param.options.export_directory = pwd; 
export_dynamics_functions(param)



%**********************************************
% 4. Open Loop Simulation
%**********************************************
%% 4.1
% From Section V.C
%Simulate rolling using either the full dynamics or the partial dynamics 

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

% Simulate rolling dyanmics
param.sim.states_t = run_dynamic_rolling_simulation(param);



%**********************************************
% 5. Visualization
%**********************************************
%% 5.1 Visualize rolling trajectory 

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



%**********************************************
% 6. Analyze Results 
%**********************************************
%% 6.1
% TODO: 
% - Remove extra scrp code from the end 
analyze_rolling_trajectory(param, param.sim.states_t)


%% Ball on plate case
% if strcmp(param.geo.model,'plane-sphere')
%     qdq_t0 = states(1,[7:11,18:22])';
%     s_X1_t0 = states(1,[1:6])'; 
%     Tso1_t0 = param.functions.fTso1(qdq_t0,s_X1_t0);
%     Rso1_t0 = Tso1_t0(1:3,1:3); 
% 
% 
%     a = param.geo.R1; 
%     M = param.dynamics.inertial.mass1;
%     I1 = param.dynamics.inertial.I1;
%     C = I1(1,1);
%     C_v2 = 2/5*M*a^2;
%     w = b_omega_o2(1,3);
%     r1 = [q_t(1,3:4)';0];
%     V1 = Rso1_t0*Vo_t(4:6,1); 
%     z = [0,0,1]';
%     %V1 = w*inv((1+M*a^2*inv(C)))
%     r0 = r1 - (1 + M*a^2/C)/w * VecToso3(V1)*z
% 
%     % T to complete circle = ~14.67 s) 
%     2*pi/(1/((1 + M*a^2/C)/w))
% 
% 
%     % w/(1+M*a^2/C)*(3+1/3-1)
%     % M*a^2/C/w*9.81*sin(0.1)
%     %
%     x0=0;
%     y0=0;
%     omega_c = 2/7*w
%     period = 2*pi/omega_c
%     cross(V1,[0;0;w])/w^2
%     r_c = [x0 - V1(2)/w; y0 - V1(1)/w; 0]
% 
% 
%     % Plot x(t) and y(t) of ball
%     %o2_p1_t = Rso1_t0' * s_p1_t; 
% 
%     Vdrift = 5/2*9.81/w*sin(0.1)
%     figure(9);  clf; 
%     plot(s_pco_t(1,:), s_pco_t(2,:),'.')
%     axis equal
%     figure(11);  clf; 
%     plot(t,s_pco_t(1,:)-Vdrift*t, t,s_pco_t(2,:),'.')
% end


%% 
toc














