function export_dynamics_functions(param)

export_directory = param.options.export_directory; % Export functions to the home directory 
is_optimize = param.options.is_simplify; % Should we optimize the exported functions? 

syms t_ real
states_ = param.variables.states_;
controls_ = param.variables.dVh_;

if param.options.is_fast_dynamics
    % Export functions for fast dynamics
    % This method avoids a large symbolic matrix inversion
    q_ = param.variables.q_;
    qdq_ = [q_; param.variables.dq_];
    states_hand_ = param.variables.states_hand_; 
        
    % Second Order Kinematics
    matlabFunction(subs(param.kinematics.second_order_kinematics_,param.bodies.P_,param.bodies.P),...
        'File',[export_directory '\autoGen_f_second_order_kinematics'],'Optimize',is_optimize,...
        'Outputs', {'dYdt'}, 'Vars',{t_,qdq_,param.variables.Alpha_});
    
    % Pure Rolling Constraint
    matlabFunction(param.kinematics.alpha_z_,'File',[export_directory '\autoGen_f_alpha_z_pure_rolling'],...
    'Optimize',is_optimize,'Vars',{qdq_});
    
    % Hand Dynamics
    matlabFunction(param.dynamics.dxddx_hand,'file',[export_directory, '\autoGen_f_hand_dynamics'],...
        'Optimize',is_optimize,'Vars',{t_, states_hand_, controls_});
    
    % Object Dynamics
    matlabFunction(param.dynamics.K5_,'File',[export_directory '\autoGen_f_K5'],...
        'Optimize',is_optimize,'Vars',{q_});
    matlabFunction(param.dynamics.K6_,'File',[export_directory '\autoGen_f_K6'],...
        'Optimize',is_optimize,'Vars',{states_});
    matlabFunction(param.dynamics.Ad_Toh_,'File',[export_directory '\autoGen_f_Ad_Toh'],...
        'Optimize',is_optimize,'Vars',{q_});
    
else
    warning('ENTER FULL DYNAMICS DERIVATION HERE')
    matlabFunction(param.dynamics.full_dynamics_,'file',[export_directory, '\autoGen_f_full_dynamics'],'Vars',...
                                                    {t_, states_, controls_});            
end













end