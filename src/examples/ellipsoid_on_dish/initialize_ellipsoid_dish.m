% initialize_ellipsoid_dish.m
% Zack Woodruff
% 1/8/2021

% This code initializes the ellipsoid object and the dish (ellipsoid) hand
% and returns them in the param structure

function param = initialize_ellipsoid_dish()
disp('Initializing ball and plate surface parameterizations...')

%% 1.1.1  Symbolic Variables 
    syms uo_ vo_ uh_ vh_ psi_ real 
    syms duo_ dvo_ duh_ dvh_ dpsi_ real
    syms dduo_ ddvo_ dduh_ ddvh_ ddpsi_ real
    param.variables.Uo_=[uo_; vo_];
    param.variables.Uh_=[uh_; vh_];
    param.variables.q_ = [uo_; vo_; uh_; vh_; psi_];
    param.variables.dq_ = [duo_; dvo_; duh_; dvh_; dpsi_];
    param.variables.ddq_ = [dduo_; ddvo_; dduh_; ddvh_; ddpsi_];
    
    
%% 1.1.2 Object Geometry and Inertial Properties
%     syms radius_o_ real
%     param.bodies.object.parameters_o_ = [radius_o_]; % object symbolic varible(s)
%     param.bodies.object.parameters_o = [2]; % object symbolic varible value(s) (old 0.035)
%     param.bodies.object.fo_=[radius_o_*sin(uo_)*cos(vo_);radius_o_*sin(uo_)*sin(vo_);radius_o_*cos(uo_)]; % object shape parameterization (sphere)
%     param.functions.ffo=matlabFunction(subs(param.bodies.object.fo_,...
%                                             param.bodies.object.parameters_o_,...
%                                             param.bodies.object.parameters_o),...
%                                             'Vars',param.variables.Uo_); % object shape parameterization function
%     param.bodies.object.u_range_o = [0, pi]; % uo limits
%     param.bodies.object.v_range_o = [-pi, pi]; % vo limits
%     assumeAlso(uo_>param.bodies.object.u_range_o(1) & uo_<param.bodies.object.u_range_o(2)) % add limits to sym variable assumptions
%     assumeAlso(vo_>param.bodies.object.v_range_o(1) & vo_<param.bodies.object.v_range_o(2)) % add limits to sym variable assumptions
% 
%     % Inertia terms
%     param.bodies.object.mass_o = 1; %kg  (old 0.216)
%     radius_o = param.bodies.object.parameters_o; 
%     inertia_o = 2/5 * param.bodies.object.mass_o * radius_o^2; % rot inertia of a sphere 
%     Io = eye(3) * inertia_o;
%     Mo = eye(3) * param.bodies.object.mass_o; 
%     param.bodies.object.Go=[Io, zeros(3);...
%                             zeros(3) , Mo]; % spatial inertia matrix 

syms rho_oa_ rho_ob_ rho_oc_ real
param.bodies.object.parameters_o_ = [rho_oa_; rho_ob_; rho_oc_];
param.bodies.object.parameters_o = [1,1,1.5]';
param.bodies.object.fo_=[rho_oa_*sin(uo_)*cos(vo_);rho_oa_*sin(uo_)*sin(vo_);rho_oc_*cos(uo_)];% ellipsoid shape description
param.functions.ffo=matlabFunction(subs(param.bodies.object.fo_,param.bodies.object.parameters_o_,param.bodies.object.parameters_o),'Vars',param.variables.Uo_);
param.bodies.object.u_range_o = [0, pi];
param.bodies.object.v_range_o = [-pi, pi];
assumeAlso(uo_>param.bodies.object.u_range_o(1) & uo_<param.bodies.object.u_range_o(2))
assumeAlso(vo_>param.bodies.object.v_range_o(1) & vo_<param.bodies.object.v_range_o(2))

% Inertia terms
%mass_o = 0.216; %kg
param.bodies.object.mass_o = 1; %kg
R1 = param.bodies.object.parameters_o(1);
R2 = param.bodies.object.parameters_o(2);
R3 = param.bodies.object.parameters_o(3);
Io = 1/5 * param.bodies.object.mass_o * diag([(R2^2+R3^2),(R1^2+R3^2), (R1^2+R2^2)]);
Mo = eye(3) * param.bodies.object.mass_o; 
param.bodies.object.Go=[Io, zeros(3);...
                        zeros(3) , Mo];
       
       
%% 1.1.3 Hand Geometry 
%     syms side_plate_ real
%     param.bodies.hand.parameters_h_ = side_plate_; % hand symbolic varible(s)
%     param.bodies.hand.parameters_h = 5; % hand symbolic varible value(s) (old 0.2)
%     param.bodies.hand.fh_=[uh_;vh_;0*uh_]; % hand shape parameterization (plane)
%     param.functions.ffh = @(uh,vh)[uh;vh;zeros(size(uh))]; % hand shape parameterization function
%     param.bodies.hand.u_range_h = [-param.bodies.hand.parameters_h,param.bodies.hand.parameters_h]; % uh limits
%     param.bodies.hand.v_range_h = [-param.bodies.hand.parameters_h,param.bodies.hand.parameters_h]; % vh limits
%     assumeAlso(uh_>param.bodies.hand.u_range_h(1) & uh_<param.bodies.hand.u_range_h(2)) % add limits to sym variable assumptions
%     assumeAlso(vh_>param.bodies.hand.v_range_h(1) & vh_<param.bodies.hand.v_range_h(2)) % add limits to sym variable assumptions

syms rho_ha_  rho_hb_ rho_hc_ real
param.bodies.hand.parameters_h_ = [rho_ha_; rho_hb_; rho_hc_];
param.bodies.hand.parameters_h = [4,4,8]'; 
param.bodies.hand.fh_=round(roty(pi/2))*[rho_ha_*sin(uh_)*cos(vh_);rho_ha_*sin(uh_)*sin(vh_);-rho_hc_*cos(uh_)];% ellipsoid dish shape description
param.functions.ffh=matlabFunction(subs(param.bodies.hand.fh_,param.bodies.hand.parameters_h_,param.bodies.hand.parameters_h),'Vars',param.variables.Uh_);
param.bodies.hand.u_range_h = [0,pi];
param.bodies.hand.v_range_h = [-pi/2,pi/2];
assumeAlso(uh_>param.bodies.hand.u_range_h(1) & uh_<param.bodies.hand.u_range_h(2));
assumeAlso(vh_>param.bodies.hand.v_range_h(1) & vh_<param.bodies.hand.v_range_h(2));       
        
%% 1.1.4 Combined Parameters 
    % Combined parameters in the parameterizations ffo and ffh
    param.bodies.P_ = [param.bodies.object.parameters_o_; param.bodies.hand.parameters_h_]; % Symbolic parameters
    param.bodies.P  = [param.bodies.object.parameters_o; param.bodies.hand.parameters_h]; % Parameter values 
    assumeAlso(param.bodies.P_>0) % constant parameters are all positive
    
    
%% 1.1.5 Friction Parameters
    param.bodies.mu_s = 1; % Sliding coefficient of friction
    param.bodies.mu_spin = 1; % Spinning coefficient of friction
    

%% Return
disp('    DONE.');
end










