function [wdes,u_law] = control2(u,L,udes,mass)
    %optimal input equation (6)
    u_law = u*udes;
    %compute velocities for rotors
    wdes = desvelocities(u_law,L,mass);
end 

function [wdes] = desvelocities(u_law,L,mass)
global n kf km g 
wdes = [];
    kf = 6.11e-8;km = 1.5e-9; %lift constant, drag constant 
    %equation 2.7 mellinger dissertation u_law = [Aa][wdes]
    Aa = [kf kf kf kf;0 kf*L 0 -kf*L;-kf*L 0 kf*L 0;km -km km -km];
    
    for i=1:n
        wh = sqrt(u_law(4*i-3)/(4*kf));
        wdesi = [1 0 -1 1;1 1 0 -1;1 0 1 1;1 -1 0 -1]*(u_law(4*i-3:4*i)+[wh;0;0;0]);
        wdes = [wdes;wdesi];
    end
    
%     %computes angular velocity of rotor 1,2,3,4 of all drones 
%      for i=1:n
%          %des velocity to the power of 2 for drone i
%          b = u_law(4*i-3:4*i);
%          %wdes = Aa^-1*u_law(that corresponds to drone i)
%          wdesi = inv(Aa)*b;
%          for j=1:4
%             wdesi(j) = sqrt(abs(wdesi(j))); 
%          end
%         wdes = [wdes;wdesi];
%      end
end