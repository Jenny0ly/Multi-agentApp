function [r,dr] = numericalmethod (ddr,ddr_ant,dr_ant,Q_ant)
    global dt n th t 
    %linear velocity
    dr = dt*(ddr + ddr_ant)/2;
    %linear position  
    for i=1:n
        r(:,i) = dt*(dr + dr_ant)/2 + Q_ant(:,i);
    end
end