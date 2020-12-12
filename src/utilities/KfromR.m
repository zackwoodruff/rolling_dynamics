function K = KfromR(R, Phi_) 
%%
syms t real
syms theta_(t) beta_(t) gamma_(t)
syms dtheta_ dbeta_ dgamma_ real

Phi_t =[theta_(t) beta_(t) gamma_(t)].';
dPhi_ = [dtheta_ dbeta_ dgamma_].';

R_t = subs(R,Phi_,Phi_t);
temp = R_t.'*diff(R_t,t);
%temp = diff(R_t,t)*R_t.';
temp2=simplify(subs(temp,diff(Phi_t,t),dPhi_));
temp3=so3ToVec(temp2);
K_t = simplify(expand(jacobian(temp3,dPhi_)));
K = subs(K_t,Phi_t,Phi_);
end

