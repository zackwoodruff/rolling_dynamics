function param = derive_second_order_kinematics(param)


%% Initialize acceleration expressions
disp('Calculating Second-Order Kinematics...')
%****** put into variables field
    % Variables for second order kinematics equations
    syms alphax_ alphay_ alphaz_ real
    Alpha_ = [alphax_; alphay_; alphaz_];
    param.variables.Alpha_ = Alpha_;
    
    
%% Extract surface geometry terms from param
    q_ =param.variables.q_;
    
    Rpsi_ = param.kinematics.local_geometry.Rpsi_;
    E1=param.kinematics.local_geometry.E1; 
    
    Ho_ = param.kinematics.local_geometry.object.H_;
    sqrtGo_ = param.kinematics.local_geometry.object.sqrtG_;
    
    sqrtGh_ = param.kinematics.local_geometry.hand.sqrtG_;
    Hh_ = param.kinematics.local_geometry.hand.H_; 
    
    sigma_h_ = param.kinematics.local_geometry.hand.sigma_;
    Gamma_h_ = param.kinematics.local_geometry.hand.Gamma_;
    Gamma_h_bar_ = param.kinematics.local_geometry.hand.Gammabar_;
    Gamma_h_barbar_ = param.kinematics.local_geometry.hand.Gammabarbar_;
    Lh_bar_ = param.kinematics.local_geometry.hand.Lbar_;
    Lh_barbar_ = param.kinematics.local_geometry.hand.Lbarbar_;
    
    sigma_o_ = param.kinematics.local_geometry.object.sigma_;
    Gamma_o_ = param.kinematics.local_geometry.object.Gamma_;
    Gamma_o_bar_ = param.kinematics.local_geometry.object.Gammabar_;
    Gamma_o_barbar_ = param.kinematics.local_geometry.object.Gammabarbar_;
    Lo_ = param.kinematics.local_geometry.object.L_;
    Lo_bar_ = param.kinematics.local_geometry.object.Lbar_;
    Lo_barbar_ = param.kinematics.local_geometry.object.Lbarbar_;

    
%% Extract velocity terms from param
    dq_=param.variables.dq_;
    dUo_=dq_(1:2);
    dUh_=dq_(3:4);
    dpsi_=dq_(5);
    
    duo_=dUo_(1);
    dvo_=dUo_(2);
    duh_=dUh_(1);
    dvh_=dUh_(2);
    
    Wo_ = [duo_^2; duo_*dvo_; dvo_^2];
    Wh_ = [duh_^2; duh_*dvh_; dvh_^2];
    
    %omega_xy = omega_xy1;
    %omega_z = omega_z1; 
    
    omega_xy_ = param.kinematics.omega_rel_fdqo_(1:2); 
    omega_z_ = param.kinematics.omega_rel_fdqo_(3); 
    
    
%% Calculate K2 (velocity terms)
    %K2a
    pt1_ = inv([Rpsi_*sqrtGo_   , -sqrtGh_;...
               Rpsi_*E1*Ho_*sqrtGo_, -E1*Hh_*sqrtGh_]);   
    pt2_ = [-Rpsi_*sqrtGo_*Gamma_o_bar_;...
            Rpsi_*E1*inv(sqrtGo_)*Lo_barbar_]*Wo_;
    pt3_ = [sqrtGh_*Gamma_h_bar_;...
           -E1*inv(sqrtGh_)*Lh_barbar_]*Wh_;  
    pt4_ = [-2*omega_z_*E1*Rpsi_*sqrtGo_, zeros(2,2);...
           -omega_z_*Rpsi_*Ho_*sqrtGo_, -dpsi_*Hh_*sqrtGh_]*[dUo_;dUh_];
    pt5_ = [zeros(2,1); sigma_o_*Gamma_o_*dUo_*E1*omega_xy_];

    K2a_ = pt1_ *(pt2_+pt3_+pt4_+pt5_); 


    %K2b
    K2b_ = (E1*omega_xy_).'*Rpsi_*E1*inv(sqrtGo_)*Lo_*dUo_...
        +sigma_o_*Gamma_o_barbar_*Wo_...
        +sigma_h_*Gamma_h_barbar_*Wh_... 
        + [sigma_o_*Gamma_o_, sigma_h_*Gamma_h_] * K2a_;

    K2_=[K2a_; K2b_]; 


%% Calculate K3 (position and acceleraiton terms )

    %K3a
    E2 = [0, 0, 0, -1, 0 ,0;...
          0, 0, 0, 0, -1, 0;...
          eye(3), zeros(3,3)];

    K3a_ = [pt1_, zeros(4,1);...
           zeros(1,4), 1]*E2; 

    %K3n
    K3_= [eye(4), zeros(4,1);...
         sigma_o_*Gamma_o_, sigma_h_*Gamma_h_, -1] * K3a_; 

     
%% Second Order Rolling and Pure-Rolling Constraints 
    % Linear acceleration constraints (TODO: EQ 41?)
    axy_ = -E1*Rpsi_*sqrtGo_*dUo_*omega_z_;
    ax_ = axy_(1); 
    ay_ = axy_(2);
    az_ = Lo_bar_*Wo_+Lh_bar_*Wh_+2*(E1'*omega_xy_)'*Rpsi_*sqrtGo_*dUo_; % Non-breaking contact constraint 
    
    % Rotational acceleration constraints (pure-rolling) (TODO: EQ 45?)
    %alpha_z = ((E1*omega_xy_).'*Rpsi_*E1*inv(sqrtGo_)*Lo_*dUo_);

    
%% Relative Acceleration Expressions?
% Should the relative acceleration expressions K4 be calculated here?


%% Save into functions and symbolic fields  

    % Full second-order kinematics equation
    second_order_kinematics_ = (K2_+K3_*[Alpha_;ax_; ay_; az_]); 
    
    if param.options.is_simplify
        param.kinematics.K2_ = simplify(K2_);
        param.kinematics.K3_ = simplify(K3_);
        param.kinematics.second_order_kinematics_ = simplify(second_order_kinematics_);
        param.kinematics.axyz_ = simplify([ax_;ay_;az_]);
        %alpha_z = simplify(alpha_z);
    else
        param.kinematics.K3_ = (K3_);
        param.kinematics.K2_ = (K2_);
        param.kinematics.second_order_kinematics_=second_order_kinematics_;
        param.kinematics.axyz_ = [ax_;ay_;az_];
        %alpha_z=alpha_z;
    end
    
    disp('    DONE: Calculating Second-Order Kinematics.')
end