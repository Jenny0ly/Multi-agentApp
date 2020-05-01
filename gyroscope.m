function [wq,drq] =  gyroscope(Euler,Euler_ant,rB,rB_ant)
    global dt 
    cx = cosd(Euler(1));sx = sind(Euler(1));
    cy = cosd(Euler(2));sy = sind(Euler(2));
    %angular rates (inertial frame)
    dphi = (Euler(1)-Euler_ant(1))/dt;
    dtheta = (Euler(2)-Euler_ant(2))/dt;
    dpsi = (Euler(3)-Euler_ant(3))/dt;
    dE = [dphi;dtheta;dpsi];
    %angular rates (body frame)
    wq = [cy 0 -cx*sy;0 1 sx;sy 0 cx*cy]*dE;
    
    %linear velocity (inertial frame)
    drq = (rB-rB_ant)/dt;
end