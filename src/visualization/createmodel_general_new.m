%createmodel_single
function outputs = createmodel_general_new(param,axis_scale,axis_radius)
if nargin<2
    axis_scale=1;
end

if nargin <3
    axis_radius = 0.1; 
end

%% Variables
npts = 50;
lineWidth = 3; % width of the velocity vector

for object_num=1:2
    if object_num==1
        ulist = linspace(param.bodies.object.u_range_o(1),param.bodies.object.u_range_o(2),npts) ;
        vlist = linspace(param.bodies.object.v_range_o(1),param.bodies.object.v_range_o(2),npts) ;
        [U,V] = meshgrid(ulist,vlist) ;
        %fxyz = matlabFunction(subs(param.geo.f1_,param.geo.P_,param.geo.P),'Vars',param.geo.U1_);
        fxyz=param.functions.ffo; 
    elseif object_num==2
        ulist = linspace(param.bodies.hand.u_range_h(1),param.bodies.hand.u_range_h(2),npts) ;
        vlist = linspace(param.bodies.hand.v_range_h(1),param.bodies.hand.v_range_h(2),npts) ;
        [U,V] = meshgrid(ulist,vlist);
        fxyz=param.functions.ffh;
        %if strcmp(param.geo.model,'plane-sphere')
        %    fxyz = @(U,V)[U;V;zeros(size(U))];
        %else
        %    fxyz = matlabFunction(subs(param.geo.f2_,param.geo.P_,param.geo.P),'Vars',param.geo.U2_);
        %end
    end

    xyz = fxyz(U(:)',V(:)');
    X=reshape(xyz(1,:),size(U));
    Y=reshape(xyz(2,:),size(U));
    Z=reshape(xyz(3,:),size(U));
    
    if object_num==1 % Build object
        outputs.x_o=X;
        outputs.y_o=Y;
        outputs.z_o=Z;
        outputs.S_o=surf(X,Y,Z,'EdgeColor','none','FaceColor','blue','FaceAlpha',0.4);
    elseif object_num==2 % Build hand 
        outputs.x_h=X;
        outputs.y_h=Y;
        outputs.z_h=Z;
        %outputs.S_h=surf(X,Y,Z,'EdgeColor','black','FaceColor','red','FaceAlpha',0.4);
        outputs.S_h=surf(X,Y,Z,'EdgeColor','none','FaceColor','red','FaceAlpha',0.4);
    end
end

% Build the coordinate system attached to object
[outputs.frame_o,outputs.surf_o] = attach_xyz_axis(axis_scale,axis_radius);
% Build the coordinate system attached to hand
[outputs.frame_h,outputs.surf_h] = attach_xyz_axis(axis_scale,axis_radius);
% Build the coordinate system attached to Co
[outputs.frame_Co,outputs.surf_Co] = attach_xyz_axis(axis_scale,axis_radius);
% Build the coordinate system attached to Ch
[outputs.frame_Ch,outputs.surf_Ch] = attach_xyz_axis(axis_scale,axis_radius);

% Init the plot of hand  twist and object twist
outputs.Line_h = line([0,0],[0,0],[0,0],'LineWidth',lineWidth,'Color','m');
outputs.Line_o = line([0,0],[0,0],[0,0],'LineWidth',lineWidth,'Color','m');
    
axis equal
camlight left;
lighting gouraud; % phong is more demanding but gives nicer results   
xlabel('x');
ylabel('y');
zlabel('z');
set(gcf,'Color','w')
end