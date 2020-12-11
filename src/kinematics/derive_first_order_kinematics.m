function param = derive_first_order_kinematics(param) 

% From Appendix B-B
disp('Calculating First-Order Kinematics...')

% Return: K1
param.kinematics.local_geometry.Rpsi_ = [cos(param.variables.q_(5)),-sin(param.variables.q_(5));-sin(param.variables.q_(5)),-cos(param.variables.q_(5))]; 
param.kinematics.local_geometry.E1 = [0,-1;1,0];
param.kinematics.local_geometry.object.H_tilda_ = param.kinematics.local_geometry.Rpsi_ * param.kinematics.local_geometry.object.H_ * param.kinematics.local_geometry.Rpsi_;

% Initializing velocity expressions
syms omegax_ omegay_ omegaz_ real
param.variables.omegaxy_ = [omegax_;omegay_];
param.variables.Omega_ = [omegax_; omegay_; omegaz_];

% make new function, extract these from param
    sqrtGo_ = param.kinematics.local_geometry.object.sqrtG_; 
    Rpsi_ = param.kinematics.local_geometry.Rpsi_;
    Ho_tilda_ = param.kinematics.local_geometry.object.H_tilda_; 
    Hh_ = param.kinematics.local_geometry.hand.H_; 

    sqrtGh_ = param.kinematics.local_geometry.hand.sqrtG_;

    sigma_h_ = param.kinematics.local_geometry.hand.sigma_;
    Gamma_h_ = param.kinematics.local_geometry.hand.Gamma_;

    sigma_o_ = param.kinematics.local_geometry.object.sigma_;
    Gamma_o_ = param.kinematics.local_geometry.object.Gamma_;
    E1 = param.kinematics.local_geometry.E1; 
    
    
    K1o_ = inv(sqrtGo_)*Rpsi_*inv(Ho_tilda_ + Hh_)*E1;
    K1h_ = inv(sqrtGh_)*inv(Ho_tilda_ + Hh_)*E1;
    K1_psi_ = sigma_o_*Gamma_o_*K1o_ + sigma_h_*Gamma_h_*K1h_;
    K1_ = [K1o_,    zeros(2,1);...
          K1h_,    zeros(2,1);...
          K1_psi_, -1];
    if param.options.is_simplify
        K1_ = simplify(K1_);
    end
    
    % MAYBE? Expressions for first order rolling constraints on dq_
    % rotational velocities in terms of contact coordinates
    dUo_ = param.variables.dq_(1:2);
    dUh_ = param.variables.dq_(3:4);
    dpsi_ = param.variables.dq_(5);
    omega_xy1_ = E1\(Ho_tilda_+Hh_)*Rpsi_'*sqrtGo_*dUo_;
    omega_xy2_ = E1\(Ho_tilda_+Hh_)*sqrtGh_*dUh_;
    omegaz_ = sigma_o_*Gamma_o_*dUo_+sigma_h_*Gamma_h_*dUh_-dpsi_;
    omega_z1_ = omegaz_;
    omega_z2_ = omegaz_;
    
    %omega_xy1= (inv(K1o)*param.geo.dq_(1:2));
    %omega_z1 = (K1_psi*omega_xy1 - param.geo.dq_(5));
    
    %omega_xy2= (inv(K1h)*param.geo.dq_(3:4));
    %omega_z2 = (K1_psi*omega_xy2 - param.geo.dq_(5));
    
    omega_rel_q1_ =[omega_xy1_; omega_z1_]; 
    omega_rel_q2_ =[omega_xy2_; omega_z2_];
    
    if param.options.is_simplify
        omega_rel_q1_ = simplify(omega_rel_q1_);
        omega_rel_q2_ = simplify(omega_rel_q2_);
    end
      
%**Return K1 and K1*Omega_ and relative velocity from dq expressions
    %and functions 
    param.kinematics.K1_=K1_;
    param.kinematics.first_order_kinematics_ = K1_*param.variables.Omega_; 
    param.kinematics.omega_rel_fdqo_ = omega_rel_q1_;
    
disp('    DONE: Calculating First-Order Kinematics.')   




end