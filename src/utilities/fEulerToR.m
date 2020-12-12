function R = fEulerToR(euler_angles,sequence)  
    R = eye(3); 
    for i=1:3
       switch sequence(i)
           case 'X'
               Ri = rotx(euler_angles(i));
           case 'Y'
               Ri = roty(euler_angles(i));
           case 'Z'
               Ri = rotz(euler_angles(i));
       end
       R = Ri*R;  
    end
end


%% Test Euler angle conversions 
% euler_type = 'XYZ';
% euler_angle_num = [1;2;3];
% syms theta1_ beta1_ gamma1_ real
% Phi1_ = [theta1_ beta1_ gamma1_].';
% 
% temp=fEulerToR(Phi1_,euler_type)
% simplify(subs(RToEuler_sym(temp),Phi1_,euler_angle_num));
% vpa(ans)
% 
% temp2=fEulerToR(euler_angle_num,euler_type)
% outval=RToEuler_sym(temp2)
% fEulerToR(outval,euler_type)
% outval=RToEuler_sym(temp2)