% Shufeng Ren
% 1/22/2019
% Transpose and Rotate Models
function [xt, yt, zt] = trans_model (Trans, x, y, z)
%using this function to transpose and rotate models
xt = x;
yt = y;
zt = z;

for i=1:numel(x)
   temp=[x(i);y(i);z(i);1];
   trans_cor=Trans*temp;
   xt(i)=trans_cor(1);
   yt(i)=trans_cor(2);
   zt(i)=trans_cor(3);
end