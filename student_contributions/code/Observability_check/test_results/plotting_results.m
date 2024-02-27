%grid_size = 0.05;
grid_size = 0.003;

%load('obs_plot.mat')
load('obs_plot.mat')

%figure(1);
figure(2);
clf; 

subplot(1,2,1);
grid on; hold on; axis equal;
% plotBaseArtistic(params);
imagesc('xdata',x,'ydata',y,'cdata',lui)
colorbar;
title("lui");
xlabel('x[m]');
ylabel('y[m]');
xlim([-grid_size, grid_size])
ylim([-grid_size, grid_size])

subplot(1,2,2);
grid on; hold on; axis equal;
% plotBaseArtistic(params);
imagesc('xdata',x,'ydata',y,'cdata',lecn)
colorbar;
title("lecn");
xlabel('x[m]');
ylabel('y[m]');

xlim([-grid_size, grid_size])
ylim([-grid_size, grid_size])