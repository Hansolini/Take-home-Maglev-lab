load('obs_plot.mat')

figure(1);
clf; 

subplot(2,1,1);
grid on; hold on; axis equal;
% plotBaseArtistic(params);
imagesc('xdata',x,'ydata',y,'cdata',lui)
colorbar;
xlabel('x');
ylabel('y');
xlim([-0.05, 0.05])
ylim([-0.05, 0.05])

subplot(2,1,2);
grid on; hold on; axis equal;
% plotBaseArtistic(params);
imagesc('xdata',x,'ydata',y,'cdata',lecn)
colorbar;
xlabel('x');
ylabel('y');

xlim([-0.05, 0.05])
ylim([-0.05, 0.05])