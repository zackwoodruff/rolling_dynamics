% Shufeng Ren
% 1/22/2019
% Create a coordinate system using three cylinders with different colors

function [xyz_axis, surf_xyz_axis] = attach_xyz_axis(axis_scale,cylinder_width,axis_colors)
if nargin<1
    axis_scale=2;
end
if nargin<2
    cylinder_width=0.1;
end

if nargin<3
    axis_colors={'red','green','blue'};
end

[X_x,X_y,X_z]=cylinder(cylinder_width);X_z=X_z*axis_scale;
[Y_x,Y_y,Y_z]=cylinder(cylinder_width);Y_z=Y_z*axis_scale;
[Z_x,Z_y,Z_z]=cylinder(cylinder_width);Z_z=Z_z*axis_scale;

y_axis_trans = [1,0,0,0;0,cos(-pi/2),-sin(-pi/2),0;0,sin(-pi/2), cos(-pi/2),0;0,0,0,1];
x_axis_trans = [cos(pi/2),0,sin(pi/2),0;0,1,0,0;-sin(pi/2),0,cos(pi/2),0;0,0,0,1];
[X_x,X_y,X_z] = trans_model (x_axis_trans, X_x,X_y,X_z);
[Y_x,Y_y,Y_z] = trans_model (y_axis_trans, Y_x,Y_y,Y_z);
xyz_axis = {{X_x,X_y,X_z},{Y_x,Y_y,Y_z},{Z_x,Z_y,Z_z}};

x_axis = surf(xyz_axis{1}{1},xyz_axis{1}{2},xyz_axis{1}{3},'EdgeColor','none','FaceColor',axis_colors{1});
y_axis = surf(xyz_axis{2}{1},xyz_axis{2}{2},xyz_axis{2}{3},'EdgeColor','none','FaceColor',axis_colors{2});
z_axis = surf(xyz_axis{3}{1},xyz_axis{3}{2},xyz_axis{3}{3},'EdgeColor','none','FaceColor',axis_colors{3});
surf_xyz_axis={x_axis,y_axis,z_axis};

