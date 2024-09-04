% x y plane

figure(1);

x_y_plot = tiledlayout(1,2);


%% LUI
%nexttile

grid on; hold on; axis equal;
%plotBaseArtistic(params);
clims_lui = [0 100];
%imagesc('xdata',x,'ydata',y,'cdata',lui, clims_lui)

white_heat_map_1 = 100*ones(101, 101);
white_heat_map_1([47:50 52:55], [47:50 52:55]) = 40;
%white_heat_map_1([1 101], [1:101 1:101]) = 0;
%white_heat_map_1(:, [1 101]) = 0;
imagesc('xdata',x,'ydata',y,'cdata',white_heat_map_1 , clims_lui)
%plotBaseRealistic(params, modelName);

colormap hot
%colorbar;
%title("LUI");
xlabel('x [m]');
ylabel('y [m]');
xlim([-grid_size, grid_size])
ylim([-grid_size, grid_size])

%%

% LECN
nexttile

grid on; hold on; axis equal;
% plotBaseArtistic(params);
clims_lecn = [0 50];
%imagesc('xdata',x,'ydata',y,'cdata',lecn, clims_lecn)

white_heat_map_2 = 50*ones(101, 101);
white_heat_map_2([47:50 52:55], [47:50 52:55]) = 20;
imagesc('xdata',x,'ydata',y,'cdata',white_heat_map_2 , clims_lecn)
%plotBaseRealistic(params, modelName);

colorbar;
title("LECN");
xlabel('x [m]');
ylabel('y [m]');

xlim([-grid_size, grid_size])
ylim([-grid_size, grid_size])

x_y_plot.TileSpacing = 'compact';
x_y_plot.Padding = 'compact';

%% alpha beta plane
figure(2);

alpha_beta_plot = tiledlayout(1,2);


% LUI
nexttile

grid on; hold on; axis equal;

clims_lui = [0 100];
imagesc('xdata',rad2deg(x),'ydata',rad2deg(y),'cdata',(lui), clims_lui)
colormap hot
colorbar
title("LUI");
xlabel(['\alpha [' char(176) ']']);
ylabel(['\beta [' char(176) ']']);
xlim([rad2deg(-grid_size), rad2deg(grid_size)])
ylim([rad2deg(-grid_size), rad2deg(grid_size)])

% LECN
nexttile

grid on; hold on; axis equal;
clims_lecn = [0 50];
imagesc('xdata',rad2deg(x),'ydata',rad2deg(y),'cdata',(lecn), clims_lecn)
colorbar;
title("LECN");
xlabel(['\alpha [' char(176) ']']);
ylabel(['\beta [' char(176) ']']);

xlim([rad2deg(-grid_size), rad2deg(grid_size)])
ylim([rad2deg(-grid_size), rad2deg(grid_size)])

alpha_beta_plot.TileSpacing = 'compact';
alpha_beta_plot.Padding = 'compact';