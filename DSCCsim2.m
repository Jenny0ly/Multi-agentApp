%% MAIN FUNCTION DEFINED
function [zpos,zdes,force,wRq,wRb,Bx,By,Bz,Qx,Qy,Qz] = DSCCsim2 (mass,B,Q,beam,L,times,z_f,K_a)
close all
    %% GLOBAL VARIABLES
    global n g
    global t tl th tf dt 
    global path_b data1_traj data2_traj cir_cle cp square line
    global ra angle Slength distance
    global pt1 pt2
    %init time(t),lift time (tl),hovering time(th),final time(tf)    
    t = times(1);tl = times(2);th = times(3); 
    dt = 0.25;
    switch path_b
        %circle
        case 'c'
            tf = times(end)+2*pi+dt;
            ra = data1_traj; %radius 
            angle = data2_traj; %angle
        %square
        case 's'
            Slength = data1_traj; %side length
            tf = th+data2_traj;
        %line (forward and back)
        case 'l'
            distance = data1_traj; %distance forward
            tf = th+data2_traj; 
    end
    g = 9.81;
    %% DISPLAY INPUT VALUES
    % total mass 
    sentence = ['Total mass is: ', num2str(mass)];
    disp(sentence)
    % Payloads CoM in world frame 
    sentence = ['Position of CoM is: ', num2str(B)];
    disp(sentence)
    % number of drones
    col = size(Q);
    n = col(2);
    sentence = ['Number of drones: ',num2str(n)];
    disp(sentence)
    % Position of drones in world frame 
    for i=1:n
        sentence = ['Position of drone: ',num2str(Q(:,i)')];
        disp(sentence)
    end
    %initial velocity
    w_init = zeros(4*n,1);
    %% Coordinate Frames 
    %B      Body frame     (xb,yb,zb) coordinates of CoM of payload
    Eb = [0;0;0]; %euler angles for payload
    %Qi      quad_ith frame     (xqi,yqi,zqi) 
    Euler = [0;0;45]; %euler angles for quads (same for everyone)!
    %relative yaw angle of Q frame wrt to B frame
    yaw = 45; %(same orientation for everyone)!
    [Xr,wRq] = quad2(Q,B,Euler); %relative position and rotation matrix
    [wRb,bRq] = payload2(Eb,wRq,Euler);  %rotation matrices
    %% 3D view of beam 
    figure(1)
    initbeam(B,wRb) %plots position of CoM
    %plot axis of Qi frame 
    for i=1:n
        plotrefsys2(Q(:,i),wRq);
    end
    plotpayload2(beam)     
%     legend([pt1 pt2],{'CoM','Drones location'},'Location','northeast')
    
    %% initial conditions 
    wq = [0;0;0]; %angular velocity of drone 
    drq = [0;0;0]; %linear velocity of drone
    Edes_ant = [0;0;0];
    ddrB_ant = [0;0;0];
    zpos = B(3);zdes = B(3);
    force = zeros(1,n);
    cp = [B(1);B(2);z_f];
    error = zeros(3,1);
    
    Bx = B(1);By = B(2);Bz = B(3); %actual position of Beam
    Qx = zeros(n,1);Qy = zeros(n,1);Qz = zeros(n,1); %actual position of drones
    for i=1:n
        Qx(i,1) = Q(1,i);
        Qy(i,1) = Q(2,i);
        Qz(i,1) = Q(3,i);
    end
    %Dynamics in Body frame
    [A,u] = DynB2(Xr,yaw); %[uF,uMx,uMy,uMz]
    %% DECENTRALIZED CONTROLLER (using drone 1)
    for i=1:tf/dt
        %time added to simulation 
        t = 0+i*dt;
        if t == dt
            %page 8 of mellinger's paper
            rB = Q(:,1)-wRb*Xr(:,1); %position of CoM
            wb = bRq*wq;            %angular velocity of CoM
            drB = drq-cross(wb,wRb*Xr(:,1));   %linear velocity of CoM
            
            [ddrB,rT] = Settrajectory(B,rB,drB,z_f,K_a);
            [udes,Edes] = trajectoryControl(ddrB,wb,Eb,Edes_ant,mass);
            zpos(i+1) = B(3);
            zdes(i+1) = rT(3);
            Bx(i+1) = rB(1);By(i+1) = rB(2);Bz(i+1) = rB(3);
            error(3,i) = (zdes(i+1)-zpos(i+1))';
        else 
            [Q,drq] = numericalmethod(ddrB,ddrB_ant,drB_ant,Q_ant); %position and linear velocity of drone
            rB = Q(:,1)-wRb*Xr(:,1);       %position of CoM, use drone 1
            [wq,~] =  gyroscope(Eb,Eb_ant,rB,rB_ant); %angular velocity of drone
            wb = bRq*wq;            %angular velocity of CoM
            drB = drq-cross(wb,wRb*Xr(:,1));   %velocity of CoM
            [ddrB,rT] = Settrajectory(B,rB,drB,z_f,K_a);
            [udes,Edes] = trajectoryControl(ddrB,wb,Eb,Edes_ant,mass);
            if t<=th
                zpos(i+1) = rB(3);
                zdes(i+1) = rT(3);
                error(3,i) = (zdes(i+1)-zpos(i+1))';
            end
            Bx(i+1) = rB(1);By(i+1) = rB(2);Bz(i+1) = rB(3);
            ddrB_ant = ddrB;
            for k=1:n
                Qx(k,i-1) = Q(1,k);
                Qy(k,i-1) = Q(2,k);
                Qz(k,i-1) = Q(3,k);
            end
            if t>th
               cc = th/dt; 
               switch path_b
                   %circle
                   case 'c'
                       cp(:,i-cc+1) = cir_cle; 
                       error(:,i-1) = cir_cle-[Bx(i+1);By(i+1);Bz(i+1)];
                   %square
                   case 's'
                       cp(:,i-cc+1) = square; 
                       error(:,i-1) = cir_cle-[Bx(i+1);By(i+1);Bz(i+1)];
                   %line (forward and back)
                   case 'l'
                       cp(:,i-cc+1) = line; 
                       error(:,i-1) = cir_cle-[Bx(i+1);By(i+1);Bz(i+1)];
               end
            end
        end  
        
        %commanded velocity
        [wdes,u_law] = control2(u,L,udes,mass);
        
        %current speed in rotor W
        %Force and Moment by each motor of each drone
        [W,F,Mo] = motormodel2(wdes, w_init);
        
        %testing if moments and force in B are as expected
        %so far not working 
%         s = [1 1 1 1;0 L 0 -L;-L 0 L 0;km/kf -km/kf km/kf -km/kf]*F(1:4);
%         s2 = [1 1 1 1;0 L 0 -L;-L 0 L 0;km/kf -km/kf km/kf -km/kf]*F(5:8);
%         Bb = A*[s;s2];

        %animated expected trajectory
        %take off --> hover
        %trajectory2(B,wRb,beam)
        
        % settings for next step in simulation
        w_init = W;
        drB_ant = drB;
        rB_ant = rB;
        Eb_ant = Eb;
        Edes_ant = Edes;
        Q_ant = Q;
        
        for j=1:n
            force(i+1,j) = u_law(4*j-3); %desired force by drone i
        end
        
    end
    graphPos(zpos,zdes)
    graphFor(force)
    graphtrajec(Bx,By,Bz,Qx,Qy,Qz)    
    graph3dtraj(Bx,By,Bz,Qx,Qy,Qz)
    L_inf = [max(abs(error(1,:)));max(abs(error(2,:)));max(abs(error(3,:)))] %infinity norm
    [rw cl] = size(error);
    L1 = sum(abs(error),2)/(cl+1) %norm 1   sum columns sum(error,2)
    L2 = sqrt(sum(error.^2,2)/(cl+1))
%     [row, col] = find(error == L_inf) %position of max error
%     t_err = col*dt %time where max error occurs
end

function graphPos(zpos,zdes)
    global dt th
    time = 0:dt:th;
    figure(2)
    cla
    plot(time,zpos)
    grid on 
    hold on 
    plot(time,zdes,'--')
    title('Position of beam')
    legend('actual position','desired position','Location','southeast')
    xlabel('time [s]')
    ylabel('z Position [m]')
    legend('boxoff')
end

function graphFor(force)
    global dt tf n
    time = 0:dt:tf;
    figure(3)
    cla
    grid on 
    hold on 
    title('Force applied by each quadcopter')
    name_leg = [];
    for i=1:n
        plot(time,force(:,i))
        name = ['Drone ', num2str(i)]; 
        name_leg = [name_leg;name];
    end 
    legend(name_leg,'Location','southeast')
    xlabel('time [s]')
    ylabel('Force [N]')
    legend('boxoff')
end

function graphtrajec(Bx,By,Bz,Qx,Qy,Qz)
global n pt1 pt2 cp
        figure(1)
        plot3(Bx,By,Bz, '--')
        for i=1:n
           pt3 = plot3(Qx(i,:),Qy(i,:),Qz(i,:), '--');
        end
        %plot3(cp(1,:),cp(2,:),cp(3,:));
        legend([pt1 pt2 pt3],{'CoM','Drones location','Trajectory'},'Location','northeast')
end

function graph3dtraj(Bx,By,Bz,Qx,Qy,Qz)
global n cp th dt
    figure(4)
    plot(cp(1,:),cp(2,:),'b')
    hold on 
    grid on 
    plot(Bx,By,'--r')
    for i=1:n
       plot(Qx(i,:),Qy(i,:), ':k');
    end
    title('3D trajectory Top view')
    xlabel('x [m]')
    ylabel('y [m]')
    legend('desired trajectory', 'CoM trajectory','Drone trajectory','Location','southeast')
    legend('boxoff')
    
    figure(5)
    plot(cp(2,:),cp(3,:),'b')
    hold on 
    grid on 
    st = th/dt; %start point
    plot(By(st:end),Bz(st:end),'--r')
    for i=1:n
       plot(Qy(i,st:end),Qz(i,st:end), ':k');
    end
    title('3D trajectory Right view')
    xlabel('y [m]')
    ylabel('z [m]')
    legend('desired trajectory', 'CoM trajectory','Drone trajectory','Location','southeast')
    legend('boxoff')
end