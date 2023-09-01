function [state] = fnSimulate(xo,u_new,Horizon,dt, h_0, gamma)

state = zeros(13, Horizon-1);
state(:, 1) = xo;

for k = 1:(Horizon-1)
   
    %states
    x = state(1, k); % state
    y = state(2, k); % x_dot
    z = state(3, k); % x3
    xd = state(4, k); % x4
    yd = state(5, k);
    zd = state(6, k);
    phi = state(7, k);
    theta = state(8, k);
    psi = state(9, k);
    p = state(10, k);
    q = state(11, k);
    r = state(12, k);

    f1 = u_new(1, k);
    f2 = u_new(2, k);
    f3 = u_new(3, k);
    f4 = u_new(4, k);
  
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
    
    h1_0 = h_0(1);
    h1 = (x-2.2)^2 + (y-2.2)^2 + (z-1)^2 - 1;
    hx1 = [2*(x-2.2), 2*(y-2.2), 2*(z-1), 0, 0, 0, 0, 0, 0, 0, 0, 0];
    
    h2_0 = h_0(2);
    h2 = (x)^2 + (y+0.2)^2 + (z)^2 - 1;
    hx2 = [2*x, 2*(y+0.2), 2*z, 0, 0, 0, 0, 0, 0, 0, 0, 0];

    h3_0 = h_0(3);
    h3 = (x-3)^2 + (y)^2 + (z-0.5)^2 - 1;
    hx3 = [2*(x-3), 2*y, 2*(z-0.5), 0, 0, 0, 0, 0, 0, 0, 0, 0];
    
    gmm = gamma;
    
    Fx(13,1) = -1 / h1^2 * hx1 * Fx(1:12,1) - gmm * (state(13,k) + 1 / h1_0 - 1/h1) ...
           -1 / h2^2 * hx2 * Fx(1:12,1) - gmm * (state(13,k) + 1 / h2_0 - 1/h2) ...
           -1 / h3^2 * hx3 * Fx(1:12,1) - gmm * (state(13,k) + 1 / h3_0 - 1/h3);
    
    state(:,k+1) = state(:,k) + Fx * dt;

end
end