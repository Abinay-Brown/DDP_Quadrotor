%% Plotting the Result
time(1)=0;
for i= 2:Horizon
    time(i) =time(i-1) + dt;  
end

% subplot(2,6,1)    
% plot(Cost,'linewidth',2); 
% xlabel('Iterations','fontsize',10)
% title('Cost Function','fontsize',10);
% grid on;

subplot(2,6,1)
plot(time,x_traj(1,:),'linewidth',2);  
hold on;
plot(time,p_target(1,1)*ones(1,Horizon),'red','linewidth',2)
ylabel('x displacement (m)','fontsize',10); 
xlabel('Time in sec','fontsize',10)
hold off;
grid on;

subplot(2,6,2);hold on;
plot(time,x_traj(2,:),'linewidth',2); 
hold on;
plot(time,p_target(2,1)*ones(1,Horizon),'red','linewidth',2)
ylabel('y displacement (m)','fontsize',10);
xlabel('Time in sec','fontsize',10)
hold off;
grid on;
   
subplot(2,6,3);hold on
plot(time,x_traj(3,:),'linewidth',2); 
plot(time,p_target(3,1)*ones(1,Horizon),'red','linewidth',2)
ylabel('z displacement (m)','fontsize',10)
xlabel('Time in sec','fontsize',10)
hold off;
grid on;
   


subplot(2,6,4)
hold on
plot(time,x_traj(4,:),'linewidth',2);  
plot(time,p_target(4,1)*ones(1,Horizon),'red','linewidth',2)
ylabel('x velocity (m/s)','fontsize',10); 
xlabel('Time in sec','fontsize',10)
hold off;
grid on;

subplot(2,6,5);hold on;
plot(time,x_traj(5,:),'linewidth',2); 
hold on;
plot(time,p_target(5,1)*ones(1,Horizon),'red','linewidth',2)
ylabel('y velocity (m/s)','fontsize',10);
xlabel('Time in sec','fontsize',10)
hold off;
grid on;
   
subplot(2,6,6);hold on
plot(time,x_traj(6,:),'linewidth',2); 
plot(time,p_target(6,1)*ones(1,Horizon),'red','linewidth',2)
ylabel('z velocity (m/s)','fontsize',10)
xlabel('Time in sec','fontsize',10)
hold off;
grid on;

subplot(2,6,7);hold on
plot(time,x_traj(7,:),'linewidth',2); 
plot(time,p_target(7,1)*ones(1,Horizon),'red','linewidth',2)
ylabel('\phi (rad)','fontsize',10)
xlabel('Time in sec','fontsize',10)
hold off;
grid on;

subplot(2,6,8);hold on
plot(time,x_traj(8,:),'linewidth',2); 
plot(time,p_target(8,1)*ones(1,Horizon),'red','linewidth',2)
ylabel('\theta (rad)','fontsize',10)
xlabel('Time in sec','fontsize',10)
hold off;
grid on;

subplot(2,6,9);hold on
plot(time,x_traj(9,:),'linewidth',2); 
plot(time,p_target(9,1)*ones(1,Horizon),'red','linewidth',2)
ylabel('\psi (rad)','fontsize',10)
xlabel('Time in sec','fontsize',10)
hold off;
grid on;

subplot(2,6,10);hold on
plot(time,x_traj(10,:),'linewidth',2); 
plot(time,p_target(10,1)*ones(1,Horizon),'red','linewidth',2)
ylabel('p (rad/s)','fontsize',10)
xlabel('Time in sec','fontsize',10)
hold off;
grid on;

subplot(2,6,11);hold on
plot(time,x_traj(11,:),'linewidth',2); 
plot(time,p_target(11,1)*ones(1,Horizon),'red','linewidth',2)
ylabel('q (rad/s)','fontsize',10)
xlabel('Time in sec','fontsize',10)
hold off;
grid on;

subplot(2,6,12);hold on
plot(time,x_traj(12,:),'linewidth',2); 
plot(time,p_target(12,1)*ones(1,Horizon),'red','linewidth',2)
ylabel('r (rad/s)','fontsize',10)
xlabel('Time in sec','fontsize',10)
hold off;
grid on;



 quadrotor_visualize(x_traj, u_traj, time);