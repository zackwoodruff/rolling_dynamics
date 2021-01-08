% derive_local_contact_geometry_expressions.m
% Zack Woodruff
% 1/8/2021

% Input:
% fi_: Symbolic surface parameterization
% Ui_: sym variables in f

% Output: 
% geometry_expressions: structure containing symbolic geometry expressions
% with expressions defined in Appendix B

function geometry_expressions_ = derive_local_contact_geometry_expressions(fi_, Ui_)

%% A. Local Geometry of Smooth Bodies Expressions
x_ci_=diff(fi_,Ui_(1)); % x direction of contact frame {c_i}
y_ci_=diff(fi_,Ui_(2)); % y direction of contact frame {c_i}
n_ci_=simplify(cross(x_ci_,y_ci_)/norm(cross(x_ci_,y_ci_))); % normal vector of contact frame {c_i}

x_hat_ = simplify(x_ci_/norm(x_ci_)); % normalized x of contact frame {c_i}
y_hat_ = simplify(y_ci_/norm(y_ci_)); % normalized y of contact frame {c_i}
z_hat_ = n_ci_; % normalized normal vector of contact frame {c_i}

g11_=simplify(dot(x_ci_,x_ci_));
g22_=simplify(dot(y_ci_,y_ci_));

% Check that parameterization is orthogonal
g12_=simplify(dot(x_ci_,y_ci_));
if g12_~= 0
   g12_
   error('Surface parameterization is not orthogonal.')
%    geometry_expressions_=[]; 
%    geometry_expressions_.y_adjusted =simplify(y_ci-(g12/g11)*x_hat); 
%    norm_y_adjusted = simplify(norm(geometry_expressions_.y_adjusted));
%    geometry_expressions_.y_vector_adjusted = simplify(geometry_expressions_.y_adjusted/norm_y_adjusted);
%    Rici_ = [x_hat,geometry_expressions_.y_vector_adjusted,z_hat];
else
   Rici_ = [x_hat_,y_hat_,z_hat_]; % EQ. (24)
end
Tici_=[Rici_,fi_;[0,0,0,1]];

G_=simplify([g11_,g12_; g12_,g22_]); % EQ. (25)
G_inverse_=inv(G_); % For orthogonal coordinate system, G is diagonal
sqrtG_ = sqrt(G_);

sigma_ = simplify(sqrt(G_(2,2)/G_(1,1)));



%% B. Second Order Kinematics Expressions
% Initialize Christoffel Functions Eq. (29)
f_christoffel1 = @(i,j,k,Ui_,xy_list) dot(diff(xy_list{i},Ui_(j)),xy_list{k});
f_christoffel = @(i,j,k,G,Ui_,xy_list) f_christoffel1(i,j,1,Ui_,xy_list)*G_inverse_(1,k)...
                                   +f_christoffel1(i,j,2,Ui_,xy_list)*G_inverse_(2,k);
                               
% Gamma Expressions 
xy_list{1} = x_ci_; 
xy_list{2} = y_ci_;

% First Order - Eq. (28)
Gamma112 = f_christoffel(1,1,2,G_,Ui_,xy_list); 
Gamma122 = f_christoffel(1,2,2,G_,Ui_,xy_list);
Gamma = [Gamma112, Gamma122];

%Second Order - Eq. (34)/(36)
Gamma111 = f_christoffel(1,1,1,G_,Ui_,xy_list);
Gamma121 = f_christoffel(1,2,1,G_,Ui_,xy_list);
Gamma212 = f_christoffel(2,1,2,G_,Ui_,xy_list);
Gamma221 = f_christoffel(2,2,1,G_,Ui_,xy_list);
Gamma222 = f_christoffel(2,2,2,G_,Ui_,xy_list);

Gammabar=[Gamma111, 2*Gamma121, Gamma221;...
          Gamma112, 2*Gamma122, Gamma222]; % Eq. (34)

Gammabarbar=[(Gamma212-Gamma111)*Gamma112+diff(Gamma112,Ui_(1));...
             (Gamma212-Gamma111)*Gamma122+(Gamma222-Gamma121)*Gamma112+diff(Gamma122,Ui_(1))+diff(Gamma112,Ui_(2));...
             (Gamma222-Gamma121)*Gamma122+diff(Gamma122,Ui_(2))]'; % Eq. (36)

% L expressions
L11 = dot(diff(x_ci_,Ui_(1)),n_ci_);
L12 = dot(diff(x_ci_,Ui_(2)),n_ci_);
L21 = dot(diff(y_ci_,Ui_(1)),n_ci_);
L22 = dot(diff(y_ci_,Ui_(2)),n_ci_);
L=[L11,L12;L21,L22]; % Eq. (26)      

Lbar=[L11, 2*L12, L22]; % Eq (35)

LbarbarA=[Gamma111*L11-diff(L11,Ui_(1));...
          Gamma111*L12+Gamma121*L11-diff(L12,Ui_(1))-diff(L11,Ui_(2));...
          Gamma121*L12-diff(L12,Ui_(2))].';

LbarbarB=[Gamma212*L21-diff(L21,Ui_(1));...
         Gamma212*L22+Gamma222*L21-diff(L22,Ui_(1))-diff(L21,Ui_(2));...
         Gamma222*L22-diff(L22,Ui_(2))].';
Lbarbar=[LbarbarA;LbarbarB]; % Eq. (37)

    
%% Expressions to export
geometry_expressions_.G_=G_;
geometry_expressions_.G_inverse_=G_inverse_;
geometry_expressions_.sqrtG_ = sqrtG_; 
geometry_expressions_.sigma_ = sigma_;

geometry_expressions_.x_hat_ = x_hat_;
geometry_expressions_.y_hat_ = y_hat_;
geometry_expressions_.z_hat_ = z_hat_;
geometry_expressions_.Tici_ = Tici_; 

geometry_expressions_.x_ = x_ci_;
geometry_expressions_.y_ = y_ci_;
geometry_expressions_.n_ = n_ci_; 

geometry_expressions_.Gamma_=Gamma;
geometry_expressions_.Gammabar_=Gammabar;
geometry_expressions_.Gammabarbar_=Gammabarbar;

geometry_expressions_.L_=L;
geometry_expressions_.Lbar_=Lbar;
geometry_expressions_.Lbarbar_=Lbarbar;

sqrtG_inverse_ = inv(sqrtG_);
geometry_expressions_.H_ = sqrtG_inverse_ * L * sqrtG_inverse_;


end