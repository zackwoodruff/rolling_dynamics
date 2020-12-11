% function initialize_ball_plate.m
% Zack Woodruff
% 11/25/2020

% This code initializes the ball (sphere) object and the plate (plane) hand
% and returns them in the param structure

function param = initialize_ball_plate(param)

%% 1.2.1  Symbolic Variables 
    syms uo_ vo_ uh_ vh_ psi_ real 
    syms duo_ dvo_ duh_ dvh_ dpsi_ real
    syms dduo_ ddvo_ dduh_ ddvh_ ddpsi_ real
    param.variables.Uo_=[uo_; vo_];
    param.variables.Uh_=[uh_; vh_];
    param.variables.q_ = [uo_; vo_; uh_; vh_; psi_];
    param.variables.dq_ = [duo_; dvo_; duh_; dvh_; dpsi_];
    param.variables.ddq_ = [dduo_; ddvo_; dduh_; ddvh_; ddpsi_];
    
    
%% 1.2.2 Object Geometry and Inertial Properties
    syms radius_o_ real
    param.bodies.object.parameters_o_ = [radius_o_];
    param.bodies.object.parameters_o = [2];
    %param.geo.rho_o = 0.035; % radius
    %param.geo.rho_o = 2; % radius
    param.bodies.object.fo_=[radius_o_*sin(uo_)*cos(vo_);radius_o_*sin(uo_)*sin(vo_);radius_o_*cos(uo_)]; % Sphere Shape description
    param.functions.ffo=matlabFunction(subs(param.bodies.object.fo_,param.bodies.object.parameters_o_,param.bodies.object.parameters_o),'Vars',param.variables.Uo_);
    param.bodies.object.u_range_o = [0, pi];
    param.bodies.object.v_range_o = [-pi, pi];
    assumeAlso(uo_>param.bodies.object.u_range_o(1) & uo_<param.bodies.object.u_range_o(2))
    assumeAlso(vo_>param.bodies.object.v_range_o(1) & vo_<param.bodies.object.v_range_o(2))

    % Inertia terms
    %mass_o = 0.216; %kg
    param.bodies.object.mass_o = 1; %kg
    radius_o = param.bodies.object.parameters_o; 
    inertia_o = 2/5 * param.bodies.object.mass_o * radius_o^2;
    Io = eye(3) * inertia_o;   
    Mo = eye(3) * param.bodies.object.mass_o; 
    param.bodies.object.Go=[Io, zeros(3);...
                            zeros(3) , Mo];

    
%% 1.2.3 Hand Geometry 
    syms side_plate_ real
    param.bodies.hand.parameters_h_ = side_plate_;
    param.bodies.hand.parameters_h = 5; %0.2;  % 5;
    param.bodies.hand.fh_=[uh_;vh_;0*uh_]; 
    param.bodies.hand.u_range_h = [-param.bodies.hand.parameters_h,param.bodies.hand.parameters_h];
    param.bodies.hand.v_range_h = [-param.bodies.hand.parameters_h,param.bodies.hand.parameters_h];
    assumeAlso(uh_>param.bodies.hand.u_range_h(1) & uh_<param.bodies.hand.u_range_h(2))
    assumeAlso(vh_>param.bodies.hand.v_range_h(1) & vh_<param.bodies.hand.v_range_h(2))
    param.functions.ffh = @(uh,vh)[uh;vh;zeros(size(uh))];


%% 1.2.4 Combined Parameters 
    % Constant parameters in the parameterizations ffo and ffh
    param.bodies.P_ = [param.bodies.object.parameters_o_; param.bodies.hand.parameters_h_];
    param.bodies.P  = [param.bodies.object.parameters_o; param.bodies.hand.parameters_h];     
    assumeAlso(param.bodies.P_>0) % constant parameters are all positive
    
end