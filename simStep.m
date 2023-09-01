function [xNew] = simStep(xin, uin, dt)

    x = xin(1); % xin
    y = xin(2); % x_dot
    z = xin(3); % x3
    xd = xin(4); % x4
    yd = xin(5);
    zd = xin(6);
    phi = xin(7);
    theta = xin(8);
    psi = xin(9);
    p = xin(10);
    q = xin(11);
    r = xin(12);

    f1 = uin(1);
    f2 = uin(2);
    f3 = uin(3);
    f4 = uin(4);
  
    % inputs
    Fx(1,1) = xd;
    Fx(2,1) = yd;
    Fx(3,1) = zd;
    Fx(4,1) = -2*sin(theta)*(f1+f2+f3+f4);
    Fx(5,1) = 2*cos(theta)*sin(phi)*(f1+f2+f3+f4);
    Fx(6,1) = 2*cos(phi)*cos(theta)*(f1+f2+f3+f4) - 9.81;
    Fx(7,1) = p + (r*cos(phi)*tan(theta))+ (q*sin(phi)*tan(theta));
    Fx(8,1) = (q*cos(phi)) - (r*sin(phi));
    Fx(9,1) = ((r*cos(phi))+(q*sin(theta)))/cos(theta);
    Fx(10,1) = (37.565*f1)-(37.565*f2)+(37.565*f3)-(37.565*f4)-(0.7188*q*r); 
    Fx(11,1) = (37.565*f3)-(37.565*f2)-(37.565*f1)+(37.565*f4)+(0.7188*p*r);
    Fx(12,1) = (3.0745*f1)-(3.0745*f2)-(3.0745*f3)+(3.0745*f4);
    
    
    xNew = xin + Fx * dt;
end