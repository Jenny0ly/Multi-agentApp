function initbeam(p)
global pt1
    grid('on')
    hold('on')
    xlabel('longitudinal position [m]')
    ylabel('lateral position [m]')
    zlabel('height [m]')

    %CoM position 
    pt1 = plot3(p(1),p(2),p(3),'cd',...
    'MarkerSize', 8,...
    'LineWidth',1,...
    'MarkerFaceColor',[0,1,1]);
end