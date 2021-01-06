function [param, frames] = analyze_rolling_trajectory(param, states)
%states = [roll_2, pitch_2, yaw_2; ...                       % Phi_2 (1:3)
%           s_p2x; s_p2y; s_p2z; ...                          % p (4:6)
%           u1; v1, u2, v2, psi; ...                          % q (7:11)
%           b_omega_o2_x; b_omega_o2_y; b_omega_o2_z; ...     % omega_b2 (12:14)
%           b_vx_o2; b_vy_o2; b_vz_o2                         % vb2 (15:17)
%           du1; dv1, du2, dv2, dpsi;];                       % dq (18:22)

% Extracting states
s_Phi_so2_t = states(:,1:3);
s_p_so2_t = states(:,4:6);
q_t = states(:,7:11);
b_omega_o2 = states(:,12:14);
b_v_o2 = states(:,15:17);
dq_t = states(:,18:22);
qdq_t = [q_t, dq_t]; 

t=param.sim.tvec; 
npts = length(t); 
P = param.bodies.P; 
P_ = param.bodies.P_;

% Plot settings
set(0,'defaulttextInterpreter','latex')
set(0,'defaultLegendInterpreter','latex')
latex_fontsize=15;
line_colors=lines;
colors_temp = linspecer(3); 
rgb = colors_temp([2,3,1],:);




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%State Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Fig 1: Plot q(t) and dq(t)
figure(1); clf
subplot(2,1,1)
h_controls_t=plot(t,q_t,'.-');
legend({'$u_1$','$v_1$','$u_2$','$v_2$','$\psi$'},...
    'Location','EastOutside','FontSize',latex_fontsize)
xlabel('$t$ (s)')
ylabel('$q(t)$')
title('Contact Coordinates q(t)')
grid on

subplot(2,1,2)
hdUt = plot(t,dq_t,'.-');
legend({'$\dot{u}_1$','$\dot{v}_1$','$\dot{u}_2$','$\dot{v}_2$','$\dot{\psi}$'},...
    'Location','EastOutside','FontSize',latex_fontsize)
xlabel('$t$ (s)')
ylabel('$\dot{q}(t)$')
title('Contact Coordinate Velocities $\dot{q}(t)$')
grid on

%for i=1:5
%   hdUt(i).Color =line_colors(10+i,:);  
%end
%pause(1); 


%% Fig 2: Plot s_Phi_so2(t) and s_p_so2(t)
% Need to add legend or labels
axis_labels.y_labels = {'rad', 'm', 'rad/s', 'm/s'}; 
axis_labels.x_labels = {'$t(s)$', '$t(s)$', '$t(s)$', '$t(s)$'}; 
plot_four(t,{s_Phi_so2_t',b_omega_o2',s_p_so2_t',b_v_o2'},...
            {'Orientation $(^s\Phi_{so_2})$','Rotational Velocity $(^b\omega_{o_2})$',...
             'Position $(^sr_{so_2})$', 'Linear Velocity $(^bV_{o_2})$'},2,axis_labels)
         
         
figure(4); clf;
subplot(2,2,1);
h21=plot(t,s_Phi_so2_t);
xlabel('$t(s)$')
ylabel('rad')
title('Orientation $(^s\Phi_{so_2})$')

subplot(2,2,2); 
h22=plot(t,b_omega_o2);
xlabel('$t(s)$')
ylabel('m')
title('Rotational Velocity $(^b\omega_{_h})$')

subplot(2,2,3); 
h23 = plot(t,s_p_so2_t);
xlabel('$t(s)$')
ylabel('rad/s')
title('Position $(^sr_{s_h})$')

subplot(2,2,4); 
h24 = plot(t,b_v_o2);
xlabel('$t(s)$')
ylabel('m/s')
title('Linear Velocity $(^bV_{_h})$')

% Update Colors to r-g-b
for j=1:3
    h21(j).Color = rgb(:,j);
    h22(j).Color = rgb(:,j);
    h23(j).Color = rgb(:,j);
    h24(j).Color = rgb(:,j);  
end

sgtitle('Hand States')   

%% Fig 3: Controls
% Accelerations of object2 
controls_t = param.sim.controls_t;
%controls_t = param.dynamics.controls_t_fine; 
figure(3); clf
subplot(2,1,1); hold on
plot(t,controls_t(1,:),'Color',rgb(1,:),'LineWidth',1.5) % roll
plot(t,controls_t(2,:),'Color',rgb(2,:),'LineWidth',1.5) % pitch
plot(t,controls_t(3,:),'Color',rgb(3,:),'LineWidth',1.5) % yaw
hleg = legend('$^b\dot{\omega}_{o_2,x}$', '$^b\dot{\omega}_{o_2,y}$', '$^b\dot{\omega}_{o_2,z}$');
set(hleg,'interpreter','latex','FontSize',15)

title('Rotational Controls on Object 2')
ylabel('Acceleration $(\mathrm{rad}/s^2)$')
xlabel('$t (s)$')

subplot(2,1,2); hold on
plot(t,controls_t(4,:),'Color',rgb(1,:),'LineWidth',1.5) %Ux
plot(t,controls_t(5,:),'Color',rgb(2,:),'LineWidth',1.5) %Uy 
plot(t,controls_t(6,:),'Color',rgb(3,:),'LineWidth',1.5) % Uz
hleg = legend('$^b\dot{V}_{o_2,x}$', '$^b\dot{V}_{o_2,y}$', '$^b\dot{V}_{o_2,z}$');
set(hleg,'interpreter','latex','FontSize',15)
title('Linear Controls on Object 2')
xlabel('$t (s)$')
ylabel('Acceleration $(m/s^2)$')


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Validity checks 
%%_%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Check slipping
% Rolling linear velocity constraint Eq. (32)
q_ = param.variables.q_;
dq_ = param.variables.dq_;

K1_ = subs(param.kinematics.K1_,P_,P);
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

% Plotting results 
figure(4); clf 
subplot(2,1,1); hold on
plot(t, noslip_t(1:2,:)')
%plot(t, (param.functions.fomegaxy(qdq_t')-param.functions.fomegaxy2(qdq_t')))
title('No Slip Constraint Eq. (32)')
xlabel('$t (s)$')
%ylabel('Contact Velocities')
subplot(2,1,2);
plot(t, no_spin_t')
title('No Spin Constraint Eq. (33)')
xlabel('$t (s)$')
ylabel('$\omega_z$')

%% Check energy 
% Checking angular momentum conservation
Vo_t = param.functions.fVo(states')
%param.dynamics.functions.fb_V_o1(P,states'); 
Vh_t = states(:,12:17)';

Gh = eye(6); 
mass_h = 1; 
% Kinetic Energy
KE_object = sum(1/2*param.bodies.object.Go*Vo_t.^2); 
KE_hand = sum(1/2*Gh*Vh_t.^2);

% Potential energy 
for i=1:npts
    q_ti = states(i,[7:11])';
    s_X1_ti = states(i,[1:6])';
    Tsh_ti = param.functions.fTsh(s_X1_ti); 
    Tso_ti = Tsh_ti*param.functions.fTho(q_ti);
    s_po_t(:,i) = Tso_ti(1:3,4);%Tso1_ti(3,4);
    
    Tsco_ti = Tsh_ti*param.functions.fThco(q_ti);   
    s_pco_t(:,i) = Tsco_ti(1:3,4);    
end
s_p2z_t = states(1:npts,6)';%states(1:npts,6)';
PE_object = param.bodies.object.mass_o*param.dynamics.gravity*s_po_t(3,:); 
PE_hand = mass_h*param.dynamics.gravity*s_p2z_t; 

% Total energy
Etotal = KE_object + KE_hand + PE_object + PE_hand; 

% Plots
figure(5); clf
subplot(2,1,1)
plot(t,[KE_object; PE_object; KE_hand; PE_hand; Etotal]')
legend('$KE_1$', '$PE_1$', '$KE_2$', '$PE_2$', '$E_\mathrm{total}$','Location','NorthEast')
title('Energy Conservation Verification')
xlabel('$t (s)$')
ylabel('Energy') 

subplot(2,1,2)
plot(t,Etotal'-Etotal(1))
xlabel('t (s)')
ylabel('$\Delta E_\mathrm{total}$') %(m/s^2) and (rad/s^2)


%% 
warning('STILL NEED TO COMPLETE THE LAST ANALYSIS CODE'); 
return 

%% Extract frictional forces
if param.dynamics.dydx_method_num ==1 % Analytical method 
    %lambda_t = zeros(4,npts);
    lambda_t = param.dynamics.functions.flambda(P,states',controls_t); 
elseif param.dynamics.dydx_method_num ==2 % C1C2 method 
    alpha_t = zeros(3,npts); 
    if strcmp(param.dynamics.rolling_type,'pure')
        lambda_t = zeros(4,npts); 
    else
        lambda_t = zeros(3,npts); 
    end
    for i=1:npts
        q_ti =q_t(i,:)';
        dq_ti =dq_t(i,:)';
        x_ti = states(i,:)';
        controls_ti = controls_t(:,i);
        C1val=param.dynamics.functions.fC1(q_ti); 
        C2val=param.dynamics.functions.fC2(x_ti,controls_ti); 
        alpha_lambda=inv(C1val)*(C2val);
        
        if strcmp(param.dynamics.rolling_type,'pure')
            alpha_t(1:2,i) = alpha_lambda(1:2);
            %alpha_t(3,i) = param.dynamics.functions.falphaz_pure_rolling([q_ti;dq_ti]);
            alpha_t(3,i) = autoGen_falphaz_pure_rolling([q_ti;dq_ti]);
            lambda_t(:,i) = alpha_lambda(3:6); 
        else
            alpha_t(:,i) = alpha_lambda(1:3); 
            lambda_t(:,i) = alpha_lambda(4:6);
        end
    end    
end

%%
figure(6); clf
subplot(2,1,1); hold on
if size(lambda_t,1)==3  
    plot(t,lambda_t(1,:),'Color',rgb(1,:),'LineWidth',1.5) % fx
    plot(t,lambda_t(2,:),'Color',rgb(2,:),'LineWidth',1.5) % fy
    plot(t,lambda_t(3,:),'Color',rgb(3,:),'LineWidth',1.5) % fz
    hleg = legend('$^{c_2}F_x$', '$^{c_2}F_y$', '$^{c_2}F_z$');
else
    plot(t,lambda_t(1,:),'Color',rgb(3,:),'LineWidth',1.5,'LineStyle',':') % tau z
    plot(t,lambda_t(2,:),'Color',rgb(1,:),'LineWidth',1.5) % fx
    plot(t,lambda_t(3,:),'Color',rgb(2,:),'LineWidth',1.5) % fy
    plot(t,lambda_t(4,:),'Color',rgb(3,:),'LineWidth',1.5) % fz
    hleg = legend('$^{c_2}\tau_z$', '$^{c_2}F_x$', '$^{c_2}F_y$', '$^{c_2}F_z$');
end
set(hleg,'interpreter','latex','FontSize',15)
title('Constraint Forces/Torques $\lambda$')
ylabel('Forces (N) or Torques (Nm)')
xlabel('$t (s)$')

% Checking friction limits 
param.dynamics.mu=1; 
mu=param.dynamics.mu; 
subplot(2,1,2); hold on
if size(lambda_t,1)==3  
    plot(t,abs(lambda_t(1,:)),'Color',rgb(1,:),'LineWidth',1.5) % fx
    plot(t,abs(lambda_t(2,:)),'Color',rgb(2,:),'LineWidth',1.5) % fy
    plot(t,lambda_t(3,:)*mu,'Color',rgb(3,:),'LineWidth',1.5) % fz
    hleg = legend('$||^{c_2}F_x||$', '$||^{c_2}F_y||$', '$^{c_2}F_z \mu$');
else
    plot(t,lambda_t(1,:),'Color',rgb(3,:),'LineWidth',1.5,'LineStyle',':') % tau z
    plot(t,abs(lambda_t(2,:)),'Color',rgb(1,:),'LineWidth',1.5) % fx
    plot(t,abs(lambda_t(3,:)),'Color',rgb(2,:),'LineWidth',1.5) % fy
    plot(t,lambda_t(4,:)*mu,'Color',rgb(3,:),'LineWidth',1.5) % fz
    hleg = legend('$^{c_2}\tau_z$', '$||^{c_2}F_x||$', '$||^{c_2}F_y||$', '$^{c_2}F_z \mu$');
end
title('Checking Friction Limits')
xlabel('$t (s)$')

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
if strcmp(param.dynamics.rolling_type,'pure')
    figure(7); clf
    subplot(4,1,1)
    plot(t, noslip_t(3,:)')
    ylabel('$\omega_z$')
    xlabel('$t$')

    subplot(4,1,2)
    plot(t,lambda_t(1,:),'Color',rgb(3,:),'LineWidth',1.5,'LineStyle',':') % tau z
    ylabel('$\tau_z$')
    xlabel('$t$')

    subplot(4,1,3);
    deltaE = Etotal-Etotal(1); 
    plot(t,deltaE)
    ylabel('$E(t)-E(0)$')
    xlabel('$t$')

    subplot(4,1,4); hold on
    plot(t,derivative(deltaE,1,2)/diff(t(1:2)))
    plot(t,noslip_t(3,:).*lambda_t(1,:),'o','MarkerSize',2)
    ylabel('$dE/dt$')
    xlabel('$t$')
    legend('KE/PE func','$\tau_z \omega_z$')
end

%% Compare domega and alpha
dt=diff(t(1:2)); 
%alphax_num=derivative(noslip_t(1:end) 
%alphay_num=noslip_t
%alphaz_num=noslip_t

omega_xy = param.functions.fomegaxy(qdq_t');
omega_z = param.functions.fomegaz(qdq_t');
alpha_num=derivative([omega_xy;omega_z],1,2)./dt; 

figure(8)
subplot(3,1,1)
plot(t,alpha_t',t, alpha_num')
legend('$\alpha_x$','$\alpha_y$','$\alpha_z$')
subplot(3,1,2)
plot(t,alpha_t(1:2,:)'-alpha_num(1:2,:)')
title('$\alpha_{xy}$ compare')
subplot(3,1,3)
plot(t,alpha_t(3,:)'-alpha_num(3,:)')
title('$\alpha_z$ compare')


%% Export
% if param.dynamics.is_export
%     for i=1:6
%         figure(i)
%         print(gcf,[param.dynamics.dir param.dynamics.export_figure_name '_figure' num2str(i)],'-dpng','-r300');
%     end
% end
    
%% Plots

set(0,'defaulttextInterpreter','tex')
set(0,'defaultLegendInterpreter','tex')

%% No slip
% qdq_ = [param.geo.U_;param.geo.dU_];
% omega_z_simp_ = simplify(param.functions.symbolic.omega_xyz_simple(3));
% out = fdiff_t_syms(omega_z_simp_,param.geo.U_,param.geo.dU_);
% out2 = subs(out+param.geo.ddU_(5),param.geo.P_,param.geo.P);
% param.dynamics.functions.falphaz_pure = matlabFunction(out2,'Vars',{qdq_;param.geo.ddU_(1:4)});
% simplify(param.kinematics2simp(5))

%% Ball on plate case
if strcmp(param.geo.model,'plane-sphere')
    qdq_t0 = states(1,[7:11,18:22])';
    s_X1_t0 = states(1,[1:6])'; 
    Tso1_t0 = param.functions.fTso1(qdq_t0,s_X1_t0);
    Rso1_t0 = Tso1_t0(1:3,1:3); 


    a = param.geo.R1; 
    M = param.dynamics.inertial.mass1;
    I1 = param.dynamics.inertial.I1;
    C = I1(1,1);
    C_v2 = 2/5*M*a^2;
    w = b_omega_o2(1,3);
    r1 = [q_t(1,3:4)';0];
    V1 = Rso1_t0*Vo_t(4:6,1); 
    z = [0,0,1]';
    %V1 = w*inv((1+M*a^2*inv(C)))
    r0 = r1 - (1 + M*a^2/C)/w * VecToso3(V1)*z

    % T to complete circle = ~14.67 s) 
    2*pi/(1/((1 + M*a^2/C)/w))


    % w/(1+M*a^2/C)*(3+1/3-1)
    % M*a^2/C/w*9.81*sin(0.1)
    %
    x0=0;
    y0=0;
    omega_c = 2/7*w
    period = 2*pi/omega_c
    cross(V1,[0;0;w])/w^2
    r_c = [x0 - V1(2)/w; y0 - V1(1)/w; 0]


    % Plot x(t) and y(t) of ball
    %o2_p1_t = Rso1_t0' * s_p1_t; 

    Vdrift = 5/2*9.81/w*sin(0.1)
    figure(9);  clf; 
    plot(s_pco_t(1,:), s_pco_t(2,:),'.')
    axis equal
    figure(11);  clf; 
    plot(t,s_pco_t(1,:)-Vdrift*t, t,s_pco_t(2,:),'.')
end

end