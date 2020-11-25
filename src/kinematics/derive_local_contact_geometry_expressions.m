% derive_local_contact_geometry_expressions.m
% Zack Woodruff
% Created: 11/25/2020

function out = derive_local_contact_geometry_expressions(f, vars)


%% A. Local Geometry of Smooth Bodies Expressions
x=diff(f,vars(1)); % x_i direction of contact frame
y=diff(f,vars(2)); % y_i direction of contact frame
n=simplify(cross(x,y)/norm(cross(x,y))); % normal vector of contact frame

x_hat = simplify(x/norm(x)); % normalized x_i of contact frame
y_hat = simplify(y/norm(y)); % normalized y_i of contact frame
z_hat = n; % normal vector of contact frame

g11=simplify(dot(x,x));
g22=simplify(dot(y,y));

% Check that parameterization is orthogonal
g12=simplify(dot(x,y));
if g12~= 0
   g12
   warning('Surface parameterization is not orthogonal.')
   out=[]; 
   out.y_adjusted =simplify(y-(g12/g11)*x_hat); 
   norm_y_adjusted = simplify(norm(out.y_adjusted));
   out.y_vector_adjusted = simplify(out.y_adjusted/norm_y_adjusted);
   Rici_ = [x_hat,out.y_vector_adjusted,z_hat];
else
   Rici_ = [x_hat,y_hat,z_hat];
end
Tici_=[Rici_,f;[0,0,0,1]];

G=simplify([g11,g12; g12,g22]);
G_inverse=inv(G); % For orthogonal coordinate system, G is diagonal
sigma = simplify(sqrt(G(2,2)/G(1,1)));

sqrtG = sqrt(G);
    
%% B. Second Order Kinematics Expressions
% Initialize Christoffel Functions
christoffel1 = @(i,j,k,vars,xlist) dot(diff(xlist{i},vars(j)),xlist{k});
christoffel = @(i,j,k,G,vars,xlist) christoffel1(i,j,1,vars,xlist)*G_inverse(1,k)...
                                   +christoffel1(i,j,2,vars,xlist)*G_inverse(2,k);
% Calculate second order kinematics expressions
xlist{1} = x; 
xlist{2} = y; 

% Gamma Expressions 
Gamma112 = christoffel(1,1,2,G,vars,xlist);
Gamma122 = christoffel(1,2,2,G,vars,xlist);
Gamma = [Gamma112, Gamma122]; 

Gamma111 = christoffel(1,1,1,G,vars,xlist);
Gamma121 = christoffel(1,2,1,G,vars,xlist);
Gamma211 = christoffel(2,1,1,G,vars,xlist);
Gamma212 = christoffel(2,1,2,G,vars,xlist);
Gamma221 = christoffel(2,2,1,G,vars,xlist);
Gamma222 = christoffel(2,2,2,G,vars,xlist);

Gammabar=[Gamma111, 2*Gamma121, Gamma221;...
          Gamma112, 2*Gamma122, Gamma222];

Gammabarbar=[(Gamma212-Gamma111)*Gamma112+diff(Gamma112,vars(1));...
             (Gamma212-Gamma111)*Gamma122+(Gamma222-Gamma121)*Gamma112+diff(Gamma122,vars(1))+diff(Gamma112,vars(2));...
             (Gamma222-Gamma121)*Gamma122+diff(Gamma122,vars(2))]';

% L expressions
L11 = dot(diff(x,vars(1)),n);
L12 = dot(diff(x,vars(2)),n);
L21 = dot(diff(y,vars(1)),n);
L22 = dot(diff(y,vars(2)),n);
L=[L11,L12;L21,L22];          

Lbar=[L11, 2*L12, L22];

LbarbarA=[Gamma111*L11-diff(L11,vars(1));...
          Gamma111*L12+Gamma121*L11-diff(L12,vars(1))-diff(L11,vars(2));...
          Gamma121*L12-diff(L12,vars(2))].';

LbarbarB=[Gamma212*L21-diff(L21,vars(1));...
         Gamma212*L22+Gamma222*L21-diff(L22,vars(1))-diff(L21,vars(2));...
         Gamma222*L22-diff(L22,vars(2))].';
Lbarbar=[LbarbarA;LbarbarB];

    
%% Variables to export
out.G_=G;
out.G_inverse_=G_inverse;
out.sqrtG_ = sqrtG; 
out.sigma_ = sigma;

out.x_hat_ = x_hat;
out.y_hat_ = y_hat;
out.z_hat_ = z_hat;
out.Tici_ = Tici_; 

out.x_ = x;
out.y_ = y;
out.n_ = n; 

out.Gamma_=Gamma;
out.Gammabar_=Gammabar;
out.Gammabarbar_=Gammabarbar;

out.L_=L;
out.Lbar_=Lbar;
out.Lbarbar_=Lbarbar;

sqrtG_inverse_ = inv(sqrtG);
out.H_ = sqrtG_inverse_ * L * sqrtG_inverse_;


end