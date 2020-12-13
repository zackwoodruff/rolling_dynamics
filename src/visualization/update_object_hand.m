% Shufeng Ren
% 2/4/2019
% Update the plot of  hand and object
% x, y, z: x, y, z coordinates of data points of the hand or object 
% surf_o_h: 1x1 Surface of function  surf
% % trans: the space configuration of the hand or object

function [] = update_object_hand(x, y, z, surf_o_h, trans )

[Xaxis_x,Xaxis_y,Xaxis_z] = trans_model(trans,x,y,z);

set(surf_o_h,'XData',Xaxis_x,'YData',Xaxis_y,'ZData',Xaxis_z);


end
