function plotrefsys2(p,R)
global pt2
    xb_axis = R(:,1)/2;
    yb_axis = R(:,2)/2;
    zb_axis = R(:,3);
    %p position vector of drone
    pt2 = plot3(p(1),p(2),p(3),'r*',...
            'MarkerSize', 8,...
            'LineWidth',1,...
            'MarkerFaceColor',[1,0,0]);
    %x axis red
    %y axis green
    %z axis blue
    plot3([p(1),p(1)+xb_axis(1)],[p(2),p(2)+xb_axis(2)],[p(3),p(3)+xb_axis(3)],'r','LineWidth',1)
    plot3([p(1),p(1)+yb_axis(1)],[p(2),p(2)+yb_axis(2)],[p(3),p(3)+yb_axis(3)],'g','LineWidth',1)
    plot3([p(1),p(1)+zb_axis(1)],[p(2),p(2)+zb_axis(2)],[p(3),p(3)+zb_axis(3)],'b','LineWidth',1)

    xmin = -3;xmax = 6.5;
    ymin = -3;ymax = 6.5;
    zmin = 0;zmax = 6.5; 
    axis([xmin xmax ymin ymax zmin zmax])

end