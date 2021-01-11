% derive_export_dyamics.m


function param = derive_export_dyamics(param)

%% 3.1
% From Section V.A
% Return K5, K6 (K7 and K8 if full derivation) 
param.dynamics.gravity = 9.81; 
param = derive_rolling_dynamics(param);



%% 3.2 Export dynamics functions
% From Section V.B
% Equations used by f_dynamics_handler.m
export_dynamics_functions(param)

end 
