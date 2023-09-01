function  [l0,l_x,l_xx,l_u,l_uu,l_ux] = fnCost(x, u,R, Q,dt, p_target)

l0 = 0.5*((u- 0.5 * 9.81 / 4)' *R *(u- 0.5 * 9.81 / 4) + (x-p_target)'*Q*(x-p_target));
l_x = Q*(x-p_target);
l_xx = Q;
l_u = R * (u- (0.5*9.81/4));
l_uu = R;
l_ux = zeros(4,13);

% l_x = zeros(12,1);
% l_xx = zeros(12,12);
% l_u = R * u;
% l_uu = R;
% l_ux = zeros(4,12);

end