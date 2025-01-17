clear all
close all
clc;
data = csvread('imu_trajectory.csv');
xyz = data(:, 6:8);


%%
figure('Name','Trajectory 3D XYZ KF','NumberTitle','off');
plot3(xyz(1,1), xyz(1,2), xyz(1,3), 'ro','MarkerSize',12,'MarkerFaceColor',[1 .6 .6]);
hold on;
plot3(xyz(:, 1), xyz(:, 2), xyz(:, 3), 'b', 'LineWidth', 1.5);
hold on;
plot3(xyz(end, 1), xyz(end, 2), xyz(end, 3), 'go','MarkerSize',12,'MarkerFaceColor',[0.6 1 1]);
hold off;
grid;
axis equal;
xlabel('X [m]','fontweight','bold','fontsize',16);
ylabel('Y [m]','fontweight','bold','fontsize',16);
zlabel('Z [m]','fontweight','bold','fontsize',16);
title('Trajectory of the Lidar-IMU system in IMU Frame','fontweight','bold','fontsize',16);
legend('Start', 'Estimated IMU Trajectory', 'End', 'fontweight','bold','fontsize',16);
set(gca,'FontSize', 20);


