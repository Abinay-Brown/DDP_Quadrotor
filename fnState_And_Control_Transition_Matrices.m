function [Abar,Bbar,C,c] = fnState_And_Control_Transition_Matrices(state,u,du,dt, h_0, hx1_0, hx2_0, hx3_0, gamma)

x = state(1,1);
y = state(2,1);
z = state(3,1);
xd = state(4, 1);
yd = state(5, 1);
zd = state(6, 1);
phi = state(7, 1);
theta = state(8, 1);
psi = state(9, 1);
p = state(10, 1);
q = state(11, 1);
r = state(12, 1);

f1 = u(1,1);
f2 = u(2,1);
f3 = u(3,1);
f4 = u(4,1);

h1_0 = h_0(1);
h2_0 = h_0(2);
h3_0 = h_0(3);




sig1 = tan(theta)^2 + 1;

A = zeros(12, 12);
A(1, 4) = 1; A(2, 5) = 1; A(3, 6) = 1;

A(4, 8) = -2*cos(theta)*(f1+f2+f3+f4);

A(5, 7) =  2*cos(phi)*cos(theta)*(f1+f2+f3+f4); A(5, 8) =  -2*sin(phi)*sin(theta)*(f1+f2+f3+f4);

A(6, 7) = -2*sin(phi)*cos(theta)*(f1+f2+f3+f4); A(6, 8) =  -2*cos(phi)*sin(theta)*(f1+f2+f3+f4);

A(7, 7) = (q*cos(phi)*tan(theta))-(r*sin(phi)*tan(theta)); A(7, 8) = r*cos(phi)*sig1 + q*sin(phi)*sig1; 
A(7, 10) = 1; A(7, 11) = sin(phi)*tan(theta); A(7, 12) = cos(phi)*tan(theta);

A(8, 7) = -r*cos(phi)-q*sin(phi); A(8, 11) = cos(phi); A(8, 12) = -sin(phi);

A(9, 7) = (q*cos(phi)/cos(theta))-(r*sin(phi)/cos(theta)); 
A(9, 8) = (r*cos(phi)*sin(theta)/(cos(theta)^2)) + (q*sin(phi)*sin(theta)/(cos(theta)^2));
A(9, 11) = sin(phi)/cos(theta);
A(9, 12) = cos(phi)/cos(theta);

A(10, 11) = -0.7188*r; A(10, 12) = -0.7188*q;
A(11, 10) = 0.7188*r; A(11, 12) = 0.7188*p;


sig1 = 2*cos(theta)*sin(phi);
sig2 = 2*cos(phi)*cos(theta);
sig3 = -2*sin(theta);
B = zeros(12, 4);
B(4, 1) = sig3; B(4, 2) = sig3; B(4, 3) = sig3; B(4, 4) = sig3;
B(5, 1) = sig1; B(5, 2) = sig1; B(5, 3) = sig1; B(5, 4) = sig1;
B(6, 1) = sig2; B(6, 2) = sig2; B(6, 3) = sig2; B(6, 4) = sig2;

B(10, 1) = 37.565; B(10, 2) = -37.565; B(10, 3) = 37.565; B(10, 4) = -37.565;
B(11, 1) = -37.565; B(11, 2) = -37.565; B(11, 3) = 37.565; B(11, 4) = 37.565;
B(12, 1) = 3.0745; B(12, 2) = -3.0745; B(12, 3) = -3.0745; B(12, 4) = 3.0745;

gmm = gamma;
Az1 = -1 / h1_0^2 * hx1_0 * A - gmm * (1 / h1_0^2 * hx1_0)...
      -1 / h2_0^2 * hx2_0 * A - gmm * (1 / h2_0^2 * hx2_0)...
      -1 / h3_0^2 * hx3_0 * A - gmm * (1 / h3_0^2 * hx3_0);
Az2 = -gmm;
Bz = -1 / h1_0^2 * hx1_0 * B...
     -1 / h2_0^2 * hx2_0 * B...
     -1 / h3_0^2 * hx3_0 * B;
Abar = [A zeros(12, 1);
        Az1 Az2];
Bbar = [B;
        Bz];

end

