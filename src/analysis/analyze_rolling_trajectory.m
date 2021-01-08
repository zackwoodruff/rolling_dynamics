function analyze_rolling_trajectory(param, states_t)
% Intputs:
% param - parameters, variables, equations 
% states_t - rolling trajectory states(t) 
%  states_t = 
%  [theta, beta, gamma, ...                        % Phi_sh (:,1:3) - euler orientation {h} in {s} 
%   r_sh_x, r_sh_y, r_sh_z, ...                    % r_sh (:,4:6) - position of {h} in {s}
%   uo_, vo_, uh_, vh_, psi_,...                   % q (:,7:11) - contact coordinates
%   h_omega_sh_x, h_omega_sh_y, h_omega_sh_z, ...  % h_omega_sh (:,12:14) - rot body vel {h}
%   h_v_sh_x, h_v_sh_y, h_v_sh_z, ...              % h_v_sh (:,15:17) - linear body vel {h}
%   duo_, dvo_, duh_, dvh_, dpsi_]                 % dq (:,18:22) - contact coordinate velocities

% Extracting state trajectories 
% h_V_sh = [h_omega_sh; h_v_sh]  
Phi_sh_t = states_t(:,1:3);
r_sh_t = states_t(:,4:6);
q_t = states_t(:,7:11);
h_omega_sh_t = states_t(:,12:14);
h_v_sh_t = states_t(:,15:17);
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
h23 = plot(t,h_omega_sh_t);
xlabel('$t$ (s)')
ylabel('rad/s')
title('Rotational Velocity $(^h\omega_{sh})$')

subplot(2,2,4); 
h24 = plot(t,h_v_sh_t);
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
omega_xy_ = subs(param.kinematics.omega_rel_fdqh_(1:2),P_,P);
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



%% Fig 5: Extract frictional forces
if param.options.is_fast_dynamics %  general method 
    alpha_t = zeros(3,npts); 
    if strcmp(param.options.friction_model,'rolling')
        lambda_t = zeros(3,npts); 
    elseif strcmp(param.options.friction_model,'pure-rolling')
        lambda_t = zeros(4,npts); 
    end
    
    for i=1:npts
        q_ti =q_t(i,:)';
        dq_ti =dq_t(i,:)';
        x_ti = states_t(i,:)';
        controls_ti = h_dV_sh_t(:,i);
        
        K5 = autoGen_f_K5(q_ti);
        K6 = autoGen_f_K6(x_ti);
        Ad_Toh_ = autoGen_f_Ad_Toh(q_ti);
        alpha_lambda = inv(K5)*(K6-Ad_Toh_*controls_ti);
     
        if strcmp(param.options.friction_model,'rolling')
            alpha_t(:,i) = alpha_lambda(1:3); 
            lambda_t(:,i) = alpha_lambda(4:6);
        elseif strcmp(param.options.friction_model,'pure-rolling')
            alpha_t(1:2,i) = alpha_lambda(1:2);
            alpha_t(3,i) = autoGen_f_alpha_z_pure_rolling([q_ti;dq_ti]);
            lambda_t(:,i) = alpha_lambda(3:6); 
        end
    end
    
else % Analytical method
    lambda_t = autoGen_f_F_contact(states_t',h_dV_sh_t);
    %lambda_t = param.dynamics.functions.flambda(P,states',controls_t); 
end

%
figure(5); clf
subplot(2,1,1); hold on
if strcmp(param.options.friction_model,'rolling')
    plot(t,lambda_t(1,:),'Color',rgb(1,:),'LineWidth',1.5) % fx
    plot(t,lambda_t(2,:),'Color',rgb(2,:),'LineWidth',1.5) % fy
    plot(t,lambda_t(3,:),'Color',rgb(3,:),'LineWidth',1.5) % fz
    hleg = legend('$^{c_h}f_x$', '$^{c_h}f_y$', '$^{c_h}f_z$');
elseif strcmp(param.options.friction_model,'pure-rolling')
    plot(t,lambda_t(1,:),'Color',rgb(3,:),'LineWidth',1.5,'LineStyle',':') % tau z
    plot(t,lambda_t(2,:),'Color',rgb(1,:),'LineWidth',1.5) % fx
    plot(t,lambda_t(3,:),'Color',rgb(2,:),'LineWidth',1.5) % fy
    plot(t,lambda_t(4,:),'Color',rgb(3,:),'LineWidth',1.5) % fz
    hleg = legend('$^{c_h}\tau_z$', '$^{c_h}f_x$', '$^{c_h}f_y$', '$^{c_h}f_z$');
end
set(hleg,'interpreter','latex','FontSize',latex_fontsize)
title('Constraint Forces/Torques $\lambda$')
ylabel('Forces (N) or Torques (Nm)')
xlabel('$t$ (s)')

% Checking friction limits 
mu_s = param.bodies.mu_s; 

subplot(2,1,2); hold on
if strcmp(param.options.friction_model,'rolling')
    plot(t,abs(lambda_t(1,:))/mu_s,'Color',rgb(1,:),'LineWidth',1.5) % fx
    plot(t,abs(lambda_t(2,:))/mu_s,'Color',rgb(2,:),'LineWidth',1.5) % fy
    plot(t,abs(lambda_t(3,:)),'Color',rgb(3,:),'LineWidth',1.5) % fz
    hleg = legend('$||^{c_h}f_x||/\mu_s$', '$||^{c_h}f_y||/\mu_s$', '$||^{c_h}f_z||$');
elseif strcmp(param.options.friction_model,'pure-rolling')
    mu_spin = param.bodies.mu_spin;
    plot(t,abs(lambda_t(1,:))/mu_spin,'Color',rgb(3,:),'LineWidth',1.5,'LineStyle',':') % tau z
    plot(t,abs(lambda_t(2,:))/mu_s,'Color',rgb(1,:),'LineWidth',1.5) % fx
    plot(t,abs(lambda_t(3,:))/mu_s,'Color',rgb(2,:),'LineWidth',1.5) % fy
    plot(t,abs(lambda_t(4,:)),'Color',rgb(3,:),'LineWidth',1.5) % fz
    hleg = legend('$||^{c_h}\tau_z||/\mu_\mathrm{spin}$', '$||^{c_h}f_x|| /\mu_s$', '$||^{c_h}f_y||/\mu_s$', '$||^{c_h}f_z||$');
end
title('Checking Friction Constraints')
xlabel('$t$ (s)')



%% Fig 6: Check energy 
% Checking angular momentum conservation
Vo_t = param.functions.fVo(states_t')
%param.dynamics.functions.fb_V_o1(P,states'); 
Vh_t = states_t(:,12:17)';

Gh = eye(6); 
mass_h = 1; 
% Kinetic Energy
KE_object = sum(1/2*param.bodies.object.Go*Vo_t.^2); 
KE_hand = sum(1/2*Gh*Vh_t.^2);

% Potential energy 
for i=1:npts
    q_ti = states_t(i,[7:11])';
    s_X1_ti = states_t(i,[1:6])';
    Tsh_ti = param.functions.fTsh(s_X1_ti); 
    Tso_ti = Tsh_ti*param.functions.fTho(q_ti);
    s_po_t(:,i) = Tso_ti(1:3,4);%Tso1_ti(3,4);
    
    Tsco_ti = Tsh_ti*param.functions.fThco(q_ti);   
    s_pco_t(:,i) = Tsco_ti(1:3,4);    
end
s_p2z_t = states_t(1:npts,6)';%states(1:npts,6)';
PE_object = param.bodies.object.mass_o*param.dynamics.gravity*s_po_t(3,:); 
PE_hand = mass_h*param.dynamics.gravity*s_p2z_t; 

% Total energy
Etotal = KE_object + KE_hand + PE_object + PE_hand; 

% Plots
figure(6); clf
subplot(2,1,1)
plot(t,[KE_object; PE_object; KE_hand; PE_hand; Etotal]')
legend('$KE_o$', '$PE_o$', '$KE_h$', '$PE_h$', '$E_\mathrm{total}$','Location','NorthEast')
title('Energy Conservation Verification')
xlabel('$t$ (s)')
ylabel('Energy') 

subplot(2,1,2)
plot(t,Etotal'-Etotal(1))
xlabel('$t$ (s)')
ylabel('$\Delta E_\mathrm{total}$') %(m/s^2) and (rad/s^2)



%% End Analysis
set(0,'defaulttextInterpreter','tex')
set(0,'defaultLegendInterpreter','tex')

return 

%% Plot points that violate the friction limits on figure 6
% t_opt = param.mpp.trajectories{end}.t;
% x_opt = param.mpp.trajectories{end}.x;
% u_opt = param.mpp.trajectories{end}.u;
% if param.mpp.is_rot_only
%     Fnormal = param.dynamics.fFnormal2D_rot_only(x_opt',u_opt')';
%     Fxy = param.dynamics.fFxy2D_rot_only(x_opt',u_opt')';
%     Fxy = [Fxy, zeros(size(Fxy))]; 
% else
%     Fnormal = param.dynamics.fFnormal2D(x_opt',u_opt')';
%     Fxy = param.dynamics.fFxy2D(x_opt',u_opt')';
%     Fxy = [Fxy, zeros(size(Fxy))]; 
% end
% mu=1; % friction coefficient 
% c = [-Fnormal,Fxy.^2-Fnormal.^2*mu^2]; %c <= 0 (Force normal is therefore >0 0.
% %c = -Fnormal;
% [t_index_slip1] = find(c(:,2)>0);
% [t_index_slip2] = find(c(:,3)>0);
% 
% figure(6)
% subplot(2,1,2); hold on
% if exist('h','var')
%  delete(h);
% end
% if exist('h2','var')
%  delete(h2);
% end
% h = plot(t_opt(t_index_slip1),zeros(size(t_index_slip1)),'*r');
% h2 = plot(t_opt(t_index_slip2),zeros(size(t_index_slip2)),'og');











%% Compare dE/dT to rotational work equation dE/dt = tau_z*omega_z
% if strcmp(param.dynamics.rolling_type,'pure')
%     figure(7); clf
%     subplot(4,1,1)
%     plot(t, noslip_t(3,:)')
%     ylabel('$\omega_z$')
%     xlabel('$t$')
% 
%     subplot(4,1,2)
%     plot(t,lambda_t(1,:),'Color',rgb(3,:),'LineWidth',1.5,'LineStyle',':') % tau z
%     ylabel('$\tau_z$')
%     xlabel('$t$')
% 
%     subplot(4,1,3);
%     deltaE = Etotal-Etotal(1); 
%     plot(t,deltaE)
%     ylabel('$E(t)-E(0)$')
%     xlabel('$t$')
% 
%     subplot(4,1,4); hold on
%     plot(t,derivative(deltaE,1,2)/diff(t(1:2)))
%     plot(t,noslip_t(3,:).*lambda_t(1,:),'o','MarkerSize',2)
%     ylabel('$dE/dt$')
%     xlabel('$t$')
%     legend('KE/PE func','$\tau_z \omega_z$')
% end

%% Compare domega and alpha
% dt=diff(t(1:2)); 
% %alphax_num=derivative(noslip_t(1:end) 
% %alphay_num=noslip_t
% %alphaz_num=noslip_t
% 
% omega_xy = param.functions.fomegaxy(qdq_t');
% omega_z = param.functions.fomegaz(qdq_t');
% alpha_num=derivative([omega_xy;omega_z],1,2)./dt; 
% 
% figure(8)
% subplot(3,1,1)
% plot(t,alpha_t',t, alpha_num')
% legend('$\alpha_x$','$\alpha_y$','$\alpha_z$')
% subplot(3,1,2)
% plot(t,alpha_t(1:2,:)'-alpha_num(1:2,:)')
% title('$\alpha_{xy}$ compare')
% subplot(3,1,3)
% plot(t,alpha_t(3,:)'-alpha_num(3,:)')
% title('$\alpha_z$ compare')


%% Export
% if param.dynamics.is_export
%     for i=1:6
%         figure(i)
%         print(gcf,[param.dynamics.dir param.dynamics.export_figure_name '_figure' num2str(i)],'-dpng','-r300');
%     end
% end
    
%% Plots



%% No slip
% qdq_ = [param.geo.U_;param.geo.dU_];
% omega_z_simp_ = simplify(param.functions.symbolic.omega_xyz_simple(3));
% out = fdiff_t_syms(omega_z_simp_,param.geo.U_,param.geo.dU_);
% out2 = subs(out+param.geo.ddU_(5),param.geo.P_,param.geo.P);
% param.dynamics.functions.falphaz_pure = matlabFunction(out2,'Vars',{qdq_;param.geo.ddU_(1:4)});
% simplify(param.kinematics2simp(5))



end