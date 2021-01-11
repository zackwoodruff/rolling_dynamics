% derive_export_kinematics.m


function param = derive_kinematics(param)

%% 2.1 Contact Geometry
% From Appendix B
disp('Calculating symbolic local contact geometry expressions...')

%-2.1.1 Object Geometry:
param.kinematics.local_geometry.object = ...
    derive_local_contact_geometry_expressions(param.bodies.object.fo_,...
                                              param.variables.Uo_);
%-2.1.2 Hand Geometry:
param.kinematics.local_geometry.hand = ...
    derive_local_contact_geometry_expressions(param.bodies.hand.fh_,...
                                              param.variables.Uh_);
                                          
disp('    DONE.');


%% 2.2 First Order Kinematics
% From Appendix B-B
% Returns: Omega_, K1_, first_order_kinematics_, omega_rel_fqdq1_
%          Rpsi_, E1, Ho_tilda_

param = derive_first_order_kinematics(param);


%% 2.3 Second Order Kinematics
% From Appendix B-C
% Returns: Alpha_, K2_, K3_, second_order_kinematics_, a_roll_, alpha_z_pr_
% TODO: 
% - Test alpha_z full derivation with pure-rolling
% - Improve documentation 
param = derive_second_order_kinematics(param);



end