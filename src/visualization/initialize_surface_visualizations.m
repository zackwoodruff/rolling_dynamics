% Create the surfaces and frames for visualizing the rolling objects
function outputs = initialize_surface_visualizations(param,axis_scale,axis_radius)
if nargin<2
    axis_scale=1;
end

if nargin <3
    axis_radius = 0.1; 
end

npts = 50; % Sets the fineness of the surface visualization


%% Initialize Object
ulist = linspace(param.bodies.object.u_range_o(1),param.bodies.object.u_range_o(2),npts) ;
vlist = linspace(param.bodies.object.v_range_o(1),param.bodies.object.v_range_o(2),npts) ;
[U,V] = meshgrid(ulist,vlist);
fxyz=param.functions.ffo;

xyz = fxyz(U(:)',V(:)');
X=reshape(xyz(1,:),size(U));
Y=reshape(xyz(2,:),size(U));
Z=reshape(xyz(3,:),size(U));

outputs.x_o=X;
outputs.y_o=Y;
outputs.z_o=Z;
outputs.S_o=surf(X,Y,Z,'EdgeColor','none','FaceColor','blue','FaceAlpha',0.4);


%% Initialize Hand
ulist = linspace(param.bodies.hand.u_range_h(1),param.bodies.hand.u_range_h(2),npts) ;
vlist = linspace(param.bodies.hand.v_range_h(1),param.bodies.hand.v_range_h(2),npts) ;
[U,V] = meshgrid(ulist,vlist);
fxyz=param.functions.ffh;

xyz = fxyz(U(:)',V(:)');
X=reshape(xyz(1,:),size(U));
Y=reshape(xyz(2,:),size(U));
Z=reshape(xyz(3,:),size(U));

outputs.x_h=X;
outputs.y_h=Y;
outputs.z_h=Z;
outputs.S_h=surf(X,Y,Z,'EdgeColor','black','FaceColor','red','FaceAlpha',0.4);


%% Initilize the coordinate systems 
% Build the coordinate system attached to object
[outputs.frame_o,outputs.surf_o] = attach_xyz_axis(axis_scale,axis_radius);
% Build the coordinate system attached to hand
[outputs.frame_h,outputs.surf_h] = attach_xyz_axis(axis_scale,axis_radius);
% Build the coordinate system attached to Co
[outputs.frame_Co,outputs.surf_Co] = attach_xyz_axis(axis_scale,axis_radius);
% Build the coordinate system attached to Ch
[outputs.frame_Ch,outputs.surf_Ch] = attach_xyz_axis(axis_scale,axis_radius);
    

%% Set display options 
axis equal
camlight left;
lighting gouraud; % phong is more demanding but gives nicer results   
xlabel('x');
ylabel('y');
zlabel('z');
set(gcf,'Color','w')


end