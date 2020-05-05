function [ddrB,yawT,rT] = Settrajectory(B,Euler,rB,drB,z_f,K)
global dt t tl th tf path_b
global ra angle Slength distance
global v1 v2 cir_cle pp square line xo yo 
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
            %circle
            case 'c'
                if t == th+dt
                    %p,po,p1 points in the parallel plane 
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
            %square    
            case 's'
                vel = (4*Slength)/(tf-th);
                timeSide = Slength/vel;
                ti_me = t-th;
                %first side
                if t == th+dt
                    xo = B(1);
                    yo = B(2);   
                end
                if ti_me <= timeSide  
                    xdes = B(1);
                    ydes = -vel*ti_me+yo;
                    zdes = z_f;
                %second side
                elseif ti_me > timeSide && ti_me<=2*timeSide
                    xdes = vel*(ti_me-timeSide)+xo;
                    ydes = B(2)-Slength;
                    zdes = z_f;
                    if ti_me == 2*timeSide
                        yo = ydes;
                    end
                %third side
                elseif ti_me > 2*timeSide && ti_me<=3*timeSide
                    xdes = B(1)+Slength;
                    ydes = vel*(ti_me-2*timeSide)+yo;
                    zdes = z_f;
                    if ti_me == 3*timeSide
                        xo = xdes;
                    end
                %forth side
                else
                    xdes = -vel*(ti_me-3*timeSide)+xo;
                    ydes = B(2);
                    zdes = z_f;                 
                end         
                square = [xdes;ydes;zdes];
            %line (forward and back)
            case 'l'
                vel = 2*distance/(tf-th);
                ti_me = t-th;
                if t == th+dt
                    yo = B(2);  
                elseif ti_me == (tf-th)/2 + dt
                    yo = -vel*(tf-th)/2+yo;
                end
                if ti_me <= (tf-th)/2
                    xdes = B(1);
                    ydes = -vel*ti_me+yo;
                    zdes = z_f;
                else
                    xdes = B(1);
                    ydes = vel*ti_me/2+yo;
                    zdes = z_f;
                    if ti_me == 10
                       disp('llegue') 
                    end
                end
                line = [xdes;ydes;zdes];
        end
    end

    rT = [xdes;ydes;zdes];
    yawT = Euler(3); %desired yaw angle
    %3D trajectory (desired acceleration)
    [ddrB] = desiredacc(rT,rB,drB,K,m_p);
end


function [ddrB] = desiredacc(rT,rB,drB,K,m_p) %100Hz
global dt tl t th tf
global path_b ra angle Slength distance
global v1 v2 
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
            %circle
            case 'c'
                ti_me = t-th+pi;
                drT = -ra*sin(ti_me)*v1+ra*cos(ti_me)*v2;
            %square
            case 's'
                vel = (4*Slength)/(tf-th);
                timeSide = Slength/vel;
                ti_me = t-th;
                %first side
                if ti_me <= timeSide
                    drT = [0;-vel;0];
                %second side
                elseif ti_me> timeSide && ti_me<=2*timeSide
                    drT = [vel;0;0];
                %third side
                elseif ti_me> 2*timeSide && ti_me<=3*timeSide
                    drT = [0;vel;0];
                %forth side
                else
                    drT = [-vel;0;0];
                end
            %line (forward and back)
            case 'l'
                vel = 2*distance/(tf-th);
                ti_me = t-th;
               if ti_me <= (tf-th)/2
                   drT = [0;-vel;0];
               else
                   drT = [0;vel;0];
               end
        end        
        
    end
    if t>th
       kp = K(3);kd = K(4); 
    end
    ep = (rT - rB); %position error
    ev = (drT - drB); %velocity error
    ddrB = kp*ep + kd*ev; %desired acceleration
end