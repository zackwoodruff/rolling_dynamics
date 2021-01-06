function plot_four(t,xlist,title_list,fignum, axis_labels)
    if nargin<4 
        fignum=1; 
    end

    figure(fignum); clf;
    for i=1:4
        subplot(2,2,i)
        h=plot(t,xlist{i});
        if min(size(xlist{i})) ==3 % Sets colors to RGB for plots with 3 inputs
            colors_temp = linspecer(3); 
            rgb = colors_temp([2,3,1],:);     
            for j=1:3
                h(j).Color = rgb(:,j);
            end
        end
        title(title_list{i})
        
        if nargin==5
            xlabel(axis_labels.x_labels{i})
            ylabel(axis_labels.y_labels{i})
        end
    end
end