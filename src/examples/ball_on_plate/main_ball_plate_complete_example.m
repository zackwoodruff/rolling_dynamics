% main_ball_plate_complete_example.m
% Zack Woodruff
% 11/24/2020

% This code demonstrates derives the rolling dynamics equations for a
% ball (sphere) on a plate (plane) and does an open-loop simulation. 

% Add the src scripts to the path 
addpath(genpath('../../src/'))

clear
clc
close all

%**********************************************
%% 1. Initialization 

%% 1.1 Input Parameters
    %param.options.model = 'ball-plate';
    param.options.is_simplify = false;
    param.options.is_generate_figures = false; 
    param.options.friction_model = 'pure-rolling'; %'pure-rolling' or 'rolling';
    %param.dynamics.gravity = 9.81; 
    
%% 1.2 Initialize:
    %       -1.2.1 Rolling Configuration Variables, 
    %       -1.2.2 Object Geometry and Inertial
    %       -1.2.3 Hand Geometry 
    param = initialize_ball_plate(param);


%**********************************************
%% 2. Derive Kinematics
%% 2.1 Differential Geometry
%% 2.2 First Order Kinematics
% Return: K1

%% 2.3 Second Order Kinematics
% Return: K2, K3, K4

%**********************************************
%% 3. Derive Dynamics
% Return K5, K6 (K7 and K8 if full derivation) 


%**********************************************
%% 4. Open Loop Simulation
%Simulate using either the full dynamics or the partial dynamics 

%**********************************************
%% 5. Visualize Results


%**********************************************
%% 6. Analyze Results 


