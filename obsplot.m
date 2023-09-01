function [x] = obsplot(x_traj)

% plot spherical obstacles
x_ax(1)=2.2;  y_ax(1)=2.2;  z_ax(1)=1;   r(1)=1;
x_ax(2)=0; y_ax(2)=-0.2; z_ax(2)=0;   r(2)=1;
x_ax(3)=3;  y_ax(3)=0;    z_ax(3)=0.5;  r(3)=1;  
    
obs_info=[x_ax',y_ax',z_ax',r'];
    
num_obs=size(obs_info,1);
[sphere_x,sphere_y,sphere_z]=sphere;
    for ii=1:num_obs
    spheres= surf(sphere_x*r(ii)+x_ax(ii), sphere_y*r(ii)+y_ax(ii), sphere_z*r(ii)+z_ax(ii));
    hold on;
    spheres.EdgeColor = 'k';
    spheres.FaceColor = '#A2142F';
    spheres.LineStyle = ':';
    spheres.FaceLighting = 'flat';
    spheres.FaceAlpha= 1;
    end
  hold on;
  plot3(x_traj(1,:), x_traj(2,:), x_traj(3,:))
  axis equal;
  x =0
end