function states_t = run_dynamic_rolling_simulation(param)
disp('Simulating dynamic rolling...')

% Friction type
if strcmp(param.options.friction_model,'rolling')
    friction_model_num = 1; % (1) Rolling, 
elseif strcmp(param.options.friction_model,'pure-rolling')
    friction_model_num = 2; % (2) Pure Rolling
else
    warning('INVALID FRICTION MODEL TYPE');
end

% Check pure-rolling assumption isn't violated 
if friction_model_num ==2
    if param.sim.omega_rel0(3)~=0
        error('Initial conditions violate pure-rolling assumption (omega_rel_z = 0)')
    end
end


% Run Simulation
[~,states_t] = ode45(@(t,states) f_dynamics_handler(t,states,...
        param.sim.controls_t,...
        param.sim.tvec_u,...
        friction_model_num,...
        param.options.is_fast_dynamics),...
        param.sim.tvec,...
        param.sim.states0,...
        param.sim.ode_options);


%% Return 
disp('    DONE.');
end