% compare_plate_ball_to_analytical_solution.m
% Zack Woodruff
% 1/8/2021

% This function compares dynamic rolling results to analytical trajectory
% solution from: 
%   K. Weltner, “Stable circular orbits of freely moving balls on rotating
%   discs,” American Journal of Physics, vol. 47, no. 11, pp. 984–986, 1979.
% Outlined in Section V.C of the paper

% Input:
% param: structure containing important variables, options, and functions

% Output: 
% result.omega_c : angular velocity of the contact point about the center of the circle
% result.h_v_ho_t0 : center of circle trajectory
% result.rho_c : radius of circle trajectory
% result.v_drift : drift velocity (if param.options.is_inclined) 

function result = compare_plate_ball_to_analytical_solution(param, is_print)
disp('Comparing simulated plate ball to analytical solution...'); 

%% Extact initial states
s_X1_t0 = param.sim.states0(1:6); % Initial hand configuration 
q_t0 = param.sim.states0(7:11); % Initital contact coordinates 
[Rhco, ~] = TransToRp(param.functions.fThco(q_t0)); % R matrix 

% Initial object velocity 
Vso_t0 = param.functions.fVso(param.sim.states0); % object body twist in {o}
v_so_t0 = Vso_t0(4:6); % linear velocity of object in {o}


%% Calculate values
% Eq. (13)
omega_plate = param.sim.Vsh0(3);
omega_c = 2/7*omega_plate;

% Eq. (14) 
xy_contact_plate_t0 = q_t0(3:4);
h_r_hch_t0 = [xy_contact_plate_t0; 0]; 
% Calculate h_v_ho

h_v_ho = Rhco*v_so_t0;
h_r_hc = h_r_hch_t0 - cross(h_v_ho,[0;0;1/omega_c]); % Eq. (14) 

% Eq. (15) 
rho_c = norm(h_r_hch_t0 - h_r_hc);

% Eq. (16) 
if param.options.is_inclined
    theta = s_X1_t0(1); 
    v_drift  = 5/2 * param.dynamics.gravity / omega_plate *sin(theta);
end


%% Expressions to export
result.omega_c = omega_c;
result.h_r_hc = h_r_hc';
result.rho_c = rho_c;  

if param.options.is_inclined
    result.v_drift = v_drift; 
end

if is_print
    disp(result);
end

disp('    DONE.'); 
end



%% Old Code
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