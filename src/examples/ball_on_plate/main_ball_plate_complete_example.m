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
% From Section V.A
% Return K5, K6 (K7 and K8 if full derivation) 
param.dynamics.gravity = 9.81; 
param = derive_rolling_dynamics(param);

% TODO: 
% - add full derivation of alpha_z


%% Export dynamics functions
% From Section V.B
% TODO:
% - Add derivation of full dynamics
param.options.is_fast_dynamics = true; 
param.options.export_directory = pwd; 
export_dynamics_functions(param)

% Equations used by f_dynamics_handler.m


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
        param.sim.friction_model_num),...
        param.sim.tvec,...
        param.sim.states0,...
        param.sim.ode_options);
toc
disp('    DONE: Simulating dynamic rolling.')


%**********************************************
% 5. Visualize Results
%**********************************************
%% 5.1 

% TODO: 
% - Move visualization to new function
% - Minimize use of additional functions 

% Visualize Dynamics Sim
set(0,'defaulttextInterpreter','tex')
set(0,'defaultLegendInterpreter','tex')


 Rsh_ = fEulerToR(param.variables.states_(1:3),'XYZ');
 psh_ =  param.variables.states_(4:6);
 Tsh_ = RpToTrans(Rsh_,psh_);
 param.functions.fTsh = matlabFunction(Tsh_,'vars',{param.variables.states_(1:6)});

figure(10); clf; hold on
%objects = createmodel_general_new(param,0.03,0.005);
objects = createmodel_general_new(param);
axis equal;
%xlim([-0.2,0.2]) % xlim([-7,7])
%ylim([-0.2,0.2]) % ylim([-7,7])
%zlim([-0.1,0.1]) % zlim([-2,5])
xlim([-8,8])
ylim([-7,7])
zlim([-1,4.5])
zlim([-5,4.5])
param.dynamics.is_export=true; 
param.dynamics.export_figure_name='plate_ball_regrasp_compare';
param.dynamics.dir='D:\Box Sync\research\Box_Sync_rolling_shared\dynamic_rolling\optimization\';

view([-42,35]) %view([51,23])
figure_size = [7, 7];
set(gcf,'Units', 'inches', 'PaperSize', figure_size, 'PaperPositionMode','auto', 'Position', [0,0,figure_size])
set(gca,'FontSize',12,'TickLabelInterpreter','Latex')
xlabel('$x$ (m)', 'interpreter','latex')%,'Position',[0.0300   -0.2626   -0.1377]);
ylabel('$y$ (m)', 'interpreter','latex')%,'Position',[-0.2639    0.0134   -0.1469]);
zlabel('$z$ (m)', 'interpreter','latex')

show_contact = true; 
resolution = round(length(param.sim.tvec)/100); 
for i= 1:resolution:length(param.sim.states_t)
    q_ti = param.sim.states_t(i,7:11)';
    
    % Update hand
    Tsh_ti = param.functions.fTsh(param.sim.states_t(i,1:6)'); % Hand location {h} in world frame {s}
    Tsch_ti = Tsh_ti*param.functions.fThch(q_ti); % Hand contact location {c_h} in world frame {s}
    update_object_hand(objects.x_h, objects.y_h, objects.z_h, objects.S_h, Tsh_ti) % update hand
    update_frame(objects.frame_Ch,objects.surf_Ch,Tsch_ti) % Contact frame {c_h}
    update_frame(objects.frame_h,objects.surf_h,Tsh_ti) % {h} Frame
    
    % Update object
    Tso_ti = Tsh_ti*param.functions.fTho(q_ti); % Object location {o} in world frame {s}
    Tsco_ti = Tsh_ti*param.functions.fThco(q_ti); % Object contact location {c_o} in world frame {s}
    update_object_hand(objects.x_o, objects.y_o, objects.z_o, objects.S_o, Tso_ti) % update object
    update_frame(objects.frame_o,objects.surf_o,Tso_ti) % {o} Frame
    update_frame(objects.frame_Co,objects.surf_Co,Tsco_ti) % Contact frame {c_o}
     
    % Plot contact location trajectory in space frame {s}
    if show_contact
        if i==1
            h_contact=plot3(Tsch_ti(1,4),Tsch_ti(2,4),Tsch_ti(3,4),':k','LineWidth',2); 
        else
           h_contact.XData=[h_contact.XData, Tsch_ti(1,4)];
           h_contact.YData=[h_contact.YData, Tsch_ti(2,4)];
           h_contact.ZData=[h_contact.ZData, Tsch_ti(3,4)];
        end
    end
    title({[],['t = ' num2str((i-1)*param.sim.dt,'%.2f')]})
    drawnow
    %pause(0.1)
end
% visualize_dynamic_sim(param, dynamic_sim_states, param.dynamics.tvec)



%**********************************************
% 6. Analyze Results 
%**********************************************
%% 6.1
% TODO: 
% - Add code to analyze the output trajectory 
