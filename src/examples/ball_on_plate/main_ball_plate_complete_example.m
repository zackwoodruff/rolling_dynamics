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
%   param.geo.geometry_o = diffgeo2(param.geo.fo_,param.geo.Uo_);
param.kinematics.local_geometry.object = derive_local_contact_geometry_expressions(param.bodies.object.fo_,param.variables.Uo_);


param.kinematics.local_geometry.hand = derive_local_contact_geometry_expressions(param.bodies.hand.fh_,param.variables.Uh_);
%    param.geo.geometry_h = diffgeo2(param.geo.fh_,param.geo.Uh_);

% param.kinematics.geometry = 


% Get rid of this? 
%geometry_oh = parse_object_geometries(param.geo.geometry_o,param.geo.geometry_h);

param.kinematics.local_geometry.object
param.kinematics.local_geometry.hand
%% 2.2 First Order Kinematics
% Return: K1
%param.kinematics.local_geometry.Rpsi_ = [cos(param.variables.q_(5)),-sin(param.variables.q_(5));-sin(param.variables.q_(5)),-cos(param.variables.q_(5))]; 

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

