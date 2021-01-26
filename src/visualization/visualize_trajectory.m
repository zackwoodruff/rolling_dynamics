function visualize_trajectory(param)
disp('Visualizing rolling trajectory...')

set(0,'DefaultFigureWindowStyle','normal')
set(0,'defaulttextInterpreter','tex')
set(0,'defaultLegendInterpreter','tex')

figure(10); clf; hold on
if isfield(param.options.visualization,'frame_size')
    frame_length = param.options.visualization.frame_size(1); 
    frame_radius = param.options.visualization.frame_size(2); 
    objects = initialize_surface_visualizations(param,frame_length,frame_radius); %(param,0.03,0.005)
else
    objects = initialize_surface_visualizations(param); 
end
% Set the size of the figure 
figure_size = param.options.visualization.figure_size;
set(gcf,'Units', 'inches', 'PaperSize', figure_size, 'PaperPositionMode','auto', 'Position', [0,0,figure_size])

% Set font sizes and axis lavels 
set(gca,'FontSize',12,'TickLabelInterpreter','Latex')
%xlabel('$x$', 'interpreter','latex');%,'Position',[1.22,-9.59,-6.58])%,'Position',[0.0300   -0.2626   -0.1377]);
%ylabel('$y$', 'interpreter','latex');%,'Position',[-10.4237,1.176,-6.068])%,'Position',[-0.2639    0.0134   -0.1469]);
%zlabel('$z$', 'interpreter','latex');%,'Position',[-8.9458,8.2839,0.2088])

%xlabel('$x$', 'interpreter','latex','Position',[0.0960, -0.8064, -0.0843])%,'Position',[0.0300   -0.2626   -0.1377]);
%ylabel('$y$', 'interpreter','latex','Position',[-0.83, 0.084, -0.0923])%,'Position',[-0.2639    0.0134   -0.1469]);
%zlabel('$z$', 'interpreter','latex','Position',[-0.76, 0.75, 0.22])

xlabel(' ');
ylabel(' ');
zlabel(' ');
%set(gca,'visible','off')
%set(findall(gca, 'type', 'text'), 'visible', 'on')
set(gca,'xcolor','none')
set(gca,'ycolor','none')
set(gca,'zcolor','none')

xlim(param.options.visualization.xlim);
ylim(param.options.visualization.ylim);
zlim(param.options.visualization.zlim);
view(param.options.visualization.view);


show_contact = param.options.visualization.show_contact;
resolution = round(length(param.sim.tvec)/100);
for i= 1:resolution:length(param.sim.states_t)
    q_ti = param.sim.states_t(i,7:11)';
    
    % Update hand
    Tsh_ti = param.functions.fTsh(param.sim.states_t(i,1:6)'); % Hand location {h} in world frame {s}
    Tsch_ti = Tsh_ti*param.functions.fThch(q_ti); % Hand contact location {c_h} in world frame {s}
    update_object_hand(objects.x_h, objects.y_h, objects.z_h, objects.S_h, Tsh_ti) % update hand
    update_frame(objects.frame_Ch,objects.surf_Ch,Tsch_ti) % Contact frame {c_h}
    update_frame(objects.frame_h,objects.surf_h,Tsh_ti) % {h} Frame
    
    % Update object
    Tso_ti = Tsh_ti*param.functions.fTho(q_ti); % Object location {o} in world frame {s}
    Tsco_ti = Tsh_ti*param.functions.fThco(q_ti); % Object contact location {c_o} in world frame {s}
    update_object_hand(objects.x_o, objects.y_o, objects.z_o, objects.S_o, Tso_ti) % update object
    update_frame(objects.frame_o,objects.surf_o,Tso_ti) % {o} Frame
    update_frame(objects.frame_Co,objects.surf_Co,Tsco_ti) % Contact frame {c_o}
     
    % Plot contact location trajectory in space frame {s}
    if show_contact
        if i==1
            h_contact=plot3(Tsch_ti(1,4),Tsch_ti(2,4),Tsch_ti(3,4),':k','LineWidth',1.5); 
        else
           h_contact.XData=[h_contact.XData, Tsch_ti(1,4)];
           h_contact.YData=[h_contact.YData, Tsch_ti(2,4)];
           h_contact.ZData=[h_contact.ZData, Tsch_ti(3,4)];
        end
    end
    title({[],['t = ' num2str((i-1)*param.sim.dt,'%.2f')]})
    drawnow
    
    if param.options.visualization.is_export
        if i==1
            frames_i = 1;
        end
        frames(frames_i) = getframe(gcf);
        frames_i=frames_i+1; 
    end
end


%% Export Video 
if param.options.visualization.is_export
    filename = param.options.visualization.export_figure_name;
    dir=param.options.export_directory;
    %slowDown = 5; %1/5x normal speed 
    %FrameRate=(length(tspan)-1)/tspan(end)/slowDown; 
    %set(gcf,'Position',get(0,'ScreenSize'))
    writerObj=VideoWriter([dir '\' filename '.avi'],'Motion JPEG AVI');  
    writerObj.Quality = 100;
    writerObj.FrameRate = 1/param.sim.dt/resolution; 
    open(writerObj);
    for i=1:length(frames)
        writeVideo(writerObj,frames(i))
    end
    close(writerObj);
end

%% Return
set(0,'DefaultFigureWindowStyle','docked')
disp('    DONE.'); 
