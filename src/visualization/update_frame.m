% Shufeng Ren
% 2/4/2019
% Update the plot of frames
% Description:
% xyz_axis: 1x3 cell, each of them is the data points of a cylinder, which
% replace one  axis
% surf_xyz: 1x3 cell, each of them is 1x1 Surface, which replace the plot
% of 3 axis
% trans: the space configuration of this frame


function [] = update_frame( xyz_axis, surf_xyz_axis, trans )

[Xaxis_x,Xaxis_y,Xaxis_z] = trans_model(trans,xyz_axis{1}{1},xyz_axis{1}{2},xyz_axis{1}{3});
[Yaxis_x,Yaxis_y,Yaxis_z] = trans_model(trans,xyz_axis{2}{1},xyz_axis{2}{2},xyz_axis{2}{3});
[Zaxis_x,Zaxis_y,Zaxis_z] = trans_model(trans,xyz_axis{3}{1},xyz_axis{3}{2},xyz_axis{3}{3});

set(surf_xyz_axis{1},'XData',Xaxis_x,'YData',Xaxis_y,'ZData',Xaxis_z);
set(surf_xyz_axis{2},'XData',Yaxis_x,'YData',Yaxis_y,'ZData',Yaxis_z);
set(surf_xyz_axis{3},'XData',Zaxis_x,'YData',Zaxis_y,'ZData',Zaxis_z);

end

