% Visualize the quadcopter simulation as an animation of a 3D quadcopter.
function [h,MM] = quadrotor_visualize_w_obstacles(x_traj,u_k,time,obs_info)
%         x_traj: [12xN double]
%         u_k:    [4xN double]
%         time:   [1xN double]
 
data.x = x_traj(1:3,:);
data.vel = x_traj(4:6,:);
data.theta = x_traj(7:9,:);
data.angvel = x_traj(10:12,:);
data.t = time;
data.input = [u_k zeros(4,1)];
figure; plots = subplot(1, 1, 1);
h = subplot(plots(1));

% Create the quadcopter object. Returns a handle to
% the quadcopter
[t] = quadcopter;

% Set axis scale and labels.
axis([-6 6 -6 6 -6 6]);
axis equal
zlabel('Height (m)');
ylabel('Y Position (m)')
xlabel('X Position (m)')

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
    
% Animate the quadcopter with data from the simulation.
MM = animate(data, t, plots);
end

% Animate a quadcopter in flight, using data from the simulation.
function MM = animate(data, model, plots)
% Show frames from the animation. However, in the interest of speed,
% skip some frames to make the animation more visually appealing.
for t = 1:2:length(data.t)
    i = (t+1)/2;
    % The first, main part, is for the 3D visualization.
    subplot(plots(1));
    
    % Compute translation to correct linear position coordinates.
    dx = data.x(:, t);
    move = makehgtform('translate', dx);
    
    % Compute rotation to correct angles. Then, turn this rotation
    % into a 4x4 matrix represting this affine transformation.
    angles = data.theta(:, t);
    rotate = rotation(angles);
    rotate = [rotate zeros(3, 1); zeros(1, 3) 1];
    
    % Move the quadcopter to the right place, after putting it in the correct orientation.
    set(model,'Matrix', move * rotate);
    
    % Update the drawing.
    xmin = -6;
    xmax = 6;
    ymin = -6;
    ymax = 6;
    zmin = -6;
    zmax = 6;
    axis([xmin xmax ymin ymax zmin zmax]);
    plot3([xmin xmax xmin xmax xmax],[ymin ymax ymax ymin ymax],[zmin zmin zmin zmin zmin],'b')
    c = plot3(data.x(1,t),data.x(2,t),zmin,'r+');
    plot3(data.x(1,1:t),data.x(2,1:t),data.x(3,1:t),'k--')
    title({['Time: ' num2str(data.t(t))]})
    drawnow;
    MM(i) = getframe(gcf);
    delete(c)
    
end
%Play Movie
figure
axis off
movie(MM,1,length(data.x)/data.t(end)/2)
end


% Draw a quadcopter. Return a handle to the quadcopter object
% and an array of handles to the thrust display cylinders.
% These will be transformed during the animation to display
% relative thrust forces.
function [h] = quadcopter()
% Draw arms.
h(1) = prism(-1, -0.25/5, -0.25/5, 2, 0.5/5, 0.5/5);
h(2) = prism(-0.25/5, -1, -0.25/5, 0.5/5, 2, 0.5/5);

% Draw bulbs representing propellers at the end of each arm.
[x y z] = sphere;
x = 0.5/2.5 * x;
y = 0.5/2.5 * y;
z = 0.5/2.5 * z;
h(3) = surf(x - 1, y, z, 'EdgeColor', 'none', 'FaceColor', 'b');
h(4) = surf(x + 1, y, z, 'EdgeColor', 'none', 'FaceColor', 'b');
h(5) = surf(x, y - 1, z, 'EdgeColor', 'none', 'FaceColor', 'b');
h(6) = surf(x, y + 1, z, 'EdgeColor', 'none', 'FaceColor', 'b');

% Conjoin all quadcopter parts into one object.
t = hgtransform;
set(h, 'Parent', t);
h = t;
end

% Draw a 3D prism at (x, y, z) with width w,
% length l, and height h. Return a handle to
% the prism object.
function h = prism(x, y, z, w, l, h)
[X Y Z] = prism_faces(x, y, z, w, l, h);

faces(1, :) = [4 2 1 3];
faces(2, :) = [4 2 1 3] + 4;
faces(3, :) = [4 2 6 8];
faces(4, :) = [4 2 6 8] - 1;
faces(5, :) = [1 2 6 5];
faces(6, :) = [1 2 6 5] + 2;

for i = 1:size(faces, 1)
    h(i) = fill3(X(faces(i, :)), Y(faces(i, :)), Z(faces(i, :)), 'r'); hold on;
end

% Conjoin all prism faces into one object.
t = hgtransform;
set(h, 'Parent', t);
h = t;
end

% Compute the points on the edge of a prism at
% location (x, y, z) with width w, length l, and height h.
function [X Y Z] = prism_faces(x, y, z, w, l, h)
X = [x x x x x+w x+w x+w x+w];
Y = [y y y+l y+l y y y+l y+l];
Z = [z z+h z z+h z z+h z z+h];
end

% Compute rotation matrix for a set of angles.
function R = rotation(angles)
    phi = angles(3);
    theta = angles(2);
    psi = angles(1);

    R = zeros(3);
    R(:, 1) = [
        cos(phi) * cos(theta)
        cos(theta) * sin(phi)
        - sin(theta)
    ];
    R(:, 2) = [
        cos(phi) * sin(theta) * sin(psi) - cos(psi) * sin(phi)
        cos(phi) * cos(psi) + sin(phi) * sin(theta) * sin(psi)
        cos(theta) * sin(psi)
    ];
    R(:, 3) = [
        sin(phi) * sin(psi) + cos(phi) * cos(psi) * sin(theta)
        cos(psi) * sin(phi) * sin(theta) - cos(phi) * sin(psi)
        cos(theta) * cos(psi)
    ];
end