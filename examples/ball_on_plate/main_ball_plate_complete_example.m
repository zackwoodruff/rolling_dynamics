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
%% 1.2 Object Geometry and Inertial Properties
%% 1.3 Hand Geometry 


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


