function [wq,drq] =  gyroscope(Eb,Eb_ant,rB,rB_ant)
    global dt 
    cx = cosd(Eb(1));sx = sind(Eb(1));
    cy = cosd(Eb(2));sy = sind(Eb(2));
    %angular rates (inertial frame)
    dphi = (Eb(1)-Eb_ant(1))/dt;
    dtheta = (Eb(2)-Eb_ant(2))/dt;
    dpsi = (Eb(3)-Eb_ant(3))/dt;
    dE = [dphi;dtheta;dpsi];
    %angular rates (body frame)
    wq = [cy 0 -cx*sy;0 1 sx;sy 0 cx*cy]*dE;
    
    %linear velocity (inertial frame)
    drq = (rB-rB_ant)/dt;
end