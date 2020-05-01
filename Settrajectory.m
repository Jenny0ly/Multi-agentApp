function [ddrB,yawT,rT] = Settrajectory(B,Euler,rB,drB,z_f,K)
global dt t tl th tf path_b
global v1 v2 ra angle cir_cle pp
m_p = (z_f-B(3))/tl;    %velocity
    %lift
    if t <= tl
        xdes = B(1);
        ydes = B(2);
        zdes = m_p*t+B(3); %z = vt+z_o
    %hover    
    elseif tl<t && t<=th 
        xdes = B(1);
        ydes = B(2);
        zdes = m_p*tl+B(3); %zf = value --> zf = vtl+z_o
    else
    %circle
        switch path_b
            case 'c'
                if t == th+dt
                    %p,po,p1 points in the parallel plane
%                     ra = 1; %radius 
%                     angle = 45; 
                    %bottom point 
                    po = rB;
                    %center point
                    c_y = po(2)+ra*cosd(angle);
                    d = sqrt((ra)^2-(c_y-po(2))^2);
                    pp = [po(1);c_y;d+po(3)];
                    %upper point
                    c_y = pp(2)+ra*cosd(angle);
                    d = sqrt((ra)^2-(c_y-pp(2))^2);
                    p1 = [pp(1);c_y;d+pp(3)];
                    %v1 vector in plane
                    v1 = p1-pp;
                    v1 = v1/norm(v1);
                    %v2 normal to plane 
                    v2 = cross(p1-po,po-pp);
                    v2 = v2/norm(v2); %v1 and v2 are orthogonal unit vectors
                end 
                %parametric eq
                ti_me = t-th+pi;
                ci = pp+ra*cos(ti_me)*v1+ra*sin(ti_me)*v2;
                xdes = ci(1);
                ydes = ci(2);
                zdes = ci(3);
                
                cir_cle = [xdes;ydes;zdes];
        end
    end
    

    rT = [xdes;ydes;zdes];
    yawT = Euler(3); %desired yaw angle
    %3D trajectory (desired acceleration)
    [ddrB] = desiredacc(rT,rB,drB,K,m_p);
end


function [ddrB] = desiredacc(rT,rB,drB,K,m_p) %100Hz
global dt tl t th path_b
global v1 v2 ra
kp = K(1);kd = K(2);
    %3Dtrajectory control 
    %lift
    if t<=tl
        drT = [0;0;m_p]; %derivative of rT
    %hover
    elseif  tl<t && t<=th 
        drT = [0;0;0];  %derivative of rT
    else
        switch path_b
            case 'c'
                ti_me = t-th+pi;
                drT = -ra*sin(ti_me)*v1+ra*cos(ti_me)*v2;
        end        
        
    end
    if t>th
       kp = K(3);kd = K(4); 
    end
    ep = (rT - rB); %position error
    ev = (drT - drB); %velocity error
    ddrB = kp*ep + kd*ev; %desired acceleration
end