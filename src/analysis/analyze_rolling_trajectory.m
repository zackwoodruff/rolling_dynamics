function analyze_rolling_trajectory(param, states_t)
% Intputs:
% param - parameters, variables, equations 
% states_t - rolling trajectory states(t) 
%  states_t = 
%  [theta_h, beta_h, gamma_h, ...            % Phi_sh (:,1:3) - euler orientation {h} in {s} 
%   r_sh_x, r_sh_y, r_sh_z, ...              % r_sh (:,4:6) - position of {h} in {s}
%   uo_, vo_, uh_, vh_, psi_,...             % q (:,7:11) - contact coordinates
%   omega_sh_x, omega_sh_y, omega_sh_z, ...  % omega_sh (:,12:14) - rot body vel {h}
%   v_sh_x, v_sh_y, v_sh_z, ...              % v_sh (:,15:17) - linear body vel {h}
%   duo_, dvo_, duh_, dvh_, dpsi_]           % dq (:,18:22) - contact coordinate velocities

disp('Analyzing rolling trajectory...'); 

% Extracting state trajectories 
% h_V_sh = [h_omega_sh; h_v_sh]  
Phi_sh_t = states_t(:,1:3);
r_sh_t = states_t(:,4:6);
q_t = states_t(:,7:11);
omega_sh_t = states_t(:,12:14);
v_sh_t = states_t(:,15:17);
dq_t = states_t(:,18:22);
qdq_t = [q_t, dq_t]; 

t=param.sim.tvec; 
npts = length(t); 
P = param.bodies.P; 
P_ = param.bodies.P_;

% Plot settings
set(0,'defaulttextInterpreter','latex')
set(0,'defaultLegendInterpreter','latex')
latex_fontsize=15;
colors_temp = linspecer(3); 
rgb = colors_temp([2,3,1],:);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% State Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Fig 1: Plot contact coordinates
% q(t), dq(t)

figure(1); clf
subplot(2,1,1)
plot(t,q_t,'.-');
legend({'$u_o$','$v_o$','$u_h$','$v_h$','$\psi$'},...
    'Location','EastOutside','FontSize',latex_fontsize)
xlabel('$t$ (s)')
ylabel('$q$')
title('Contact Coordinates q(t)')
grid on

subplot(2,1,2)
hdUt = plot(t,dq_t,'.-');
legend({'$\dot{u}_o$','$\dot{v}_o$','$\dot{u}_h$','$\dot{v}_h$','$\dot{\psi}$'},...
    'Location','EastOutside','FontSize',latex_fontsize)
xlabel('$t$ (s)')
ylabel('$\dot{q}$')
title('Contact Coordinate Velocities $\dot{q}(t)$')
grid on



%% Fig 2: Plot hand states 
% Plot the hand position and velocities 
% Phi_sh(t), r_sh(t)
% h_omega_sh(t), h_v_sh(t)

figure(2); clf;
%sgtitle('Hand States')

subplot(2,2,1);
h21=plot(t,Phi_sh_t);
xlabel('$t$ (s)')
ylabel('rad')
title('Orientation $(\Phi_{sh})$')

subplot(2,2,2); 
h22=plot(t,r_sh_t);
xlabel('$t$ (s)')
ylabel('m')
title('Position $(\mathbf{r}_{sh})$')

subplot(2,2,3); 
h23 = plot(t,omega_sh_t);
xlabel('$t$ (s)')
ylabel('rad/s')
title('Rotational Velocity $(^h\omega_{sh})$')

subplot(2,2,4); 
h24 = plot(t,v_sh_t);
xlabel('$t$ (s)')
ylabel('m/s')
title('Linear Velocity $(^h\mathbf{v}_{sh})$')

% Update Colors to r-g-b
for j=1:3
    h21(j).Color = rgb(:,j);
    h22(j).Color = rgb(:,j);
    h23(j).Color = rgb(:,j);
    h24(j).Color = rgb(:,j);  
end

   

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Control Plot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Fig 3: Plot hand controls
% Commanded accelations of the hand h_dV_sh_t
h_dV_sh_t = param.sim.controls_t;

figure(3); clf
%sgtitle('Hand Controls')
subplot(2,1,1); hold on
plot(t,h_dV_sh_t(1,:),'Color',rgb(1,:),'LineWidth',1.5) % d roll
plot(t,h_dV_sh_t(2,:),'Color',rgb(2,:),'LineWidth',1.5) % d pitch
plot(t,h_dV_sh_t(3,:),'Color',rgb(3,:),'LineWidth',1.5) % d yaw
hleg = legend('$^h\dot{\omega}_{sh,x}$', '$^h\dot{\omega}_{sh,y}$', '$^h\dot{\omega}_{sh,z}$');
set(hleg,'interpreter','latex','FontSize',latex_fontsize)
title('Rotational Hand Accelerations')
ylabel('rad/s$^2$')
xlabel('$t$ (s)')

subplot(2,1,2); hold on
plot(t,h_dV_sh_t(4,:),'Color',rgb(1,:),'LineWidth',1.5) %Ux
plot(t,h_dV_sh_t(5,:),'Color',rgb(2,:),'LineWidth',1.5) %Uy 
plot(t,h_dV_sh_t(6,:),'Color',rgb(3,:),'LineWidth',1.5) % Uz
hleg = legend('$^ha_{sh,x}$', '$^ha_{sh,y}$', '$^ha_{sh,z}$');
set(hleg,'interpreter','latex','FontSize',latex_fontsize)
title('Linear Hand Accelerations')
xlabel('$t$ (s)')
ylabel('m/s$^2$')



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Validity checks 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Fig 4: Check velocity constraints are satisfied
q_ = param.variables.q_;
dq_ = param.variables.dq_;

% Rolling linear velocity constraint Eq. (32)
K1_ = subs(param.kinematics.K1_,P_,P); % Eq (31) 
K1o_ = K1_(1:2,1:2);
K1h_ = K1_(3:4,1:2);
no_slip_ = simplify([inv(K1o_), -inv(K1h_), zeros(2,1)]);
f_no_slip_=matlabFunction(no_slip_*dq_,'Vars',{[q_;dq_]}); 
noslip_t = f_no_slip_(qdq_t');


% Rolling linear velocity constraint Eq. (33)
omega_xy_ = subs(param.kinematics.omega_rel_fqdq1_(1:2),P_,P);
no_spin_ =  K1_(5,1:2)* omega_xy_ - dq_(5); 
f_no_spin_=matlabFunction(no_spin_,'Vars',{[q_;dq_]}); 
no_spin_t = f_no_spin_(qdq_t');


% Plot results
figure(4); clf 
if strcmp(param.options.friction_model,'rolling')
    plot(t, noslip_t(1:2,:)')
    title('No Slip Constraints Eq. (32)')
    xlabel('$t$ (s)')
elseif strcmp(param.options.friction_model,'pure-rolling')
    subplot(2,1,1); hold on
    plot(t, noslip_t(1:2,:)')
    title('No Slip Constraints Eq. (32)')
    xlabel('$t$ (s)')
    subplot(2,1,2);
    plot(t, no_spin_t')
    title('No Spin Constraint Eq. (33)')
    xlabel('$t$ (s)')
    ylabel('$\omega_z$')
end



%% Fig 5a: Plot contact wrench
% lambda_t contains contact wrench values that are nonzero
% rolling: lambda_t = [fx; fy; fz]
% pure rolling: lambda_t = [tau_z; fx; fy; fz]

if param.options.is_partial_dynamics %  General method 
    if strcmp(param.options.friction_model,'rolling')
        lambda_t = zeros(3,npts); 
    elseif strcmp(param.options.friction_model,'pure-rolling')
        lambda_t = zeros(4,npts); 
    end
    
    for i=1:npts
        % Get states and controls at time t(i)  
        q_ti =q_t(i,:)';
        s_ti = states_t(i,:)';
        controls_ti = h_dV_sh_t(:,i);
        
        % Calculate contact wrench using Eq. (8)/(9) 
        K5 = autoGen_f_K5(q_ti);
        K6 = autoGen_f_K6(s_ti);
        Ad_Toh_ = autoGen_f_Ad_Toh(q_ti);
        alpha_lambda = inv(K5)*(K6-Ad_Toh_*controls_ti);
     
        if strcmp(param.options.friction_model,'rolling')
            lambda_t(:,i) = alpha_lambda(4:6);
        elseif strcmp(param.options.friction_model,'pure-rolling')
            lambda_t(:,i) = alpha_lambda(3:6); 
        end
    end
    
else % Analytical method using exported contact wrench function 
    lambda_t = autoGen_f_F_contact(states_t',h_dV_sh_t);
end


% Plot contact wrench 
figure(5); clf
subplot(2,1,1); hold on
if strcmp(param.options.friction_model,'rolling')
    plot(t,lambda_t(1,:),'Color',rgb(1,:),'LineWidth',1.5) % fx
    plot(t,lambda_t(2,:),'Color',rgb(2,:),'LineWidth',1.5) % fy
    plot(t,lambda_t(3,:),'Color',rgb(3,:),'LineWidth',1.5) % fz
    hleg = legend('$f_x$', '$f_y$', '$f_z$');
    ylabel('N')
elseif strcmp(param.options.friction_model,'pure-rolling')
    plot(t,lambda_t(1,:),'Color',rgb(3,:),'LineWidth',1.5,'LineStyle',':') % tau z
    plot(t,lambda_t(2,:),'Color',rgb(1,:),'LineWidth',1.5) % fx
    plot(t,lambda_t(3,:),'Color',rgb(2,:),'LineWidth',1.5) % fy
    plot(t,lambda_t(4,:),'Color',rgb(3,:),'LineWidth',1.5) % fz
    hleg = legend('$\tau_z$', '$f_x$', '$f_y$', '$f_z$');
    ylabel('N or Nm')
end
set(hleg,'interpreter','latex','FontSize',latex_fontsize)
title('Contact Wrench $\lambda$')
xlabel('$t$ (s)')



%% Fig 5b: Checking friction limit constraints  
mu_s = param.bodies.mu_s; 

% Positive normal force constraint
if strcmp(param.options.friction_model,'rolling')
    fz_constraint_index = find(lambda_t(3,:)<0);
elseif strcmp(param.options.friction_model,'pure-rolling')
    fz_constraint_index = find(lambda_t(4,:)<0);
end
if ~isempty(fz_constraint_index)
    warning('fz constraint violated during part of simulation')
end

% No linear slip constraint
if strcmp(param.options.friction_model,'rolling')
    fx_constraint_index = find(abs(lambda_t(1,:))/mu_s> lambda_t(3,:));
    fy_constraint_index = find(abs(lambda_t(2,:))/mu_s> lambda_t(3,:));
elseif strcmp(param.options.friction_model,'pure-rolling')
    fx_constraint_index = find(abs(lambda_t(2,:))/mu_s> lambda_t(4,:));
    fy_constraint_index = find(abs(lambda_t(3,:))/mu_s> lambda_t(4,:));
end
if ~isempty(fx_constraint_index)
    warning('fx constraint violated during part of simulation')
end
if ~isempty(fy_constraint_index)
    warning('fy constraint violated during part of simulation')
end

% No spin torque constraint 
if strcmp(param.options.friction_model,'pure-rolling')
    mu_spin = param.bodies.mu_spin;
    tau_z_constraint_index = find(abs(lambda_t(1,:))/mu_spin > lambda_t(4,:));
    if ~isempty(tau_z_constraint_index)
        warning('tau_z constraint violated during part of simulation')
    end
end


subplot(2,1,2); hold on
if strcmp(param.options.friction_model,'rolling')
    fx_scaled_t = abs(lambda_t(1,:))/mu_s; 
    fy_scaled_t = abs(lambda_t(2,:))/mu_s;
    fz_t = lambda_t(3,:);
    ylabel('N')
elseif strcmp(param.options.friction_model,'pure-rolling')
    tau_z_scaled_t = abs(lambda_t(1,:))/mu_spin;
    fx_scaled_t = abs(lambda_t(2,:))/mu_s; 
    fy_scaled_t = abs(lambda_t(3,:))/mu_s;
    fz_t = lambda_t(4,:);
    ylabel('N or Nm')
end

% torque
if strcmp(param.options.friction_model,'pure-rolling')
    htauz = plot(t,tau_z_scaled_t,'Color',rgb(3,:),'LineWidth',1.5,'LineStyle',':'); % tau z
    legend_array={'$||\tau_z||/\mu_\mathrm{spin}$','$||f_x||/\mu_s$', '$||f_y||/\mu_s$', '$f_z$'};
else
    legend_array={'$||f_x||/\mu_s$', '$||f_y||/\mu_s$', '$f_z$'};
end

% forces
plot(t,fx_scaled_t,'Color',rgb(1,:),'LineWidth',1.5); % fx
plot(t,fy_scaled_t,'Color',rgb(2,:),'LineWidth',1.5); % fy
plot(t,fz_t,'Color',rgb(3,:),'LineWidth',1.5); % fz

% constraint violations
if strcmp(param.options.friction_model,'pure-rolling')
    if ~isempty(fx_constraint_index)
        plot(t(tau_z_constraint_index),tau_z_scaled_t(tau_z_constraint_index),'.b','MarkerSize',3);
        legend_array{end+1} = '$\tau_z$ violation';
    end
end
if ~isempty(fx_constraint_index)
    plot(t(fx_constraint_index),fx_scaled_t(fx_constraint_index),'*r','MarkerSize',3);
    legend_array{end+1} = '$f_x$ violation';
end
if ~isempty(fy_constraint_index)
    plot(t(fy_constraint_index),fy_scaled_t(fy_constraint_index),'*g','MarkerSize',3);
    legend_array{end+1} = '$f_y$ violation';
end
if ~isempty(fz_constraint_index)
    plot(t(fz_constraint_index),fz_t(fz_constraint_index),'*b','MarkerSize',3);
    legend_array{end+1} = '$f_z$ violation';
end     

legend(legend_array)
title('Friction Constraints')
xlabel('$t$ (s)')



%% Fig 6: Check energy 
% Checking angular momentum conservation
% note: momentum is only conserved when hand is stationary because of acceleration
% control assumption. 

o_Vso_t = param.functions.fVso(states_t');
h_Vsh_t = states_t(:,12:17)';

% Define hand inertia parameters
Gh = eye(6); 
mass_h = 1; 

% Hand inertia parameters
Go = param.bodies.object.Go; 
mass_o = param.bodies.object.mass_o;

gravity = param.dynamics.gravity;


% Kinetic Energy
KE_object = sum(1/2*Go*o_Vso_t.^2); 
KE_hand = sum(1/2*Gh*h_Vsh_t.^2);

% Potential energy 
r_so_ti = zeros(3,npts); 
for i=1:npts
    q_ti = states_t(i,[7:11])';
    hand_configuration_ti = states_t(i,1:6)';
    Tsh_ti = param.functions.fTsh(hand_configuration_ti); 
    Tso_ti = Tsh_ti*param.functions.fTho(q_ti);
    r_so_ti(:,i) = Tso_ti(1:3,4);
end

PE_object = mass_o*gravity*r_so_ti(3,:);
r_so_z_ti = states_t(1:npts,6)'; % Z coordinate of hand in space frame
PE_hand = mass_h*gravity*r_so_z_ti; 

% Total energy
Etotal = KE_object + KE_hand + PE_object + PE_hand; 


% Plot results 
figure(6); clf
% Energy contribution from object and hand KE and PE
subplot(2,1,1)
plot(t,[KE_object; PE_object; KE_hand; PE_hand; Etotal]')
legend('$KE_o$', '$PE_o$', '$KE_h$', '$PE_h$', '$E_\mathrm{total}$','Location','NorthEast')
title('Energy Conservation Verification')
xlabel('$t$ (s)')
ylabel('Energy (J)') 

% Total change in energy 
subplot(2,1,2)
plot(t,Etotal'-Etotal(1))
xlabel('$t$ (s)')
ylabel('$\Delta E_\mathrm{total}$ (J)') %(m/s^2) and (rad/s^2)



%% Export
% if param.dynamics.is_export
%     for i=1:6
%         figure(i)
%         print(gcf,[param.dynamics.dir param.dynamics.export_figure_name '_figure' num2str(i)],'-dpng','-r300');
%     end
% end



%% End Analysis
set(0,'defaulttextInterpreter','tex')
set(0,'defaultLegendInterpreter','tex')


disp('    DONE.'); 
return 