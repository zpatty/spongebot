clear all
close all
lengths = [85;75;65;55;45] - 45;

weights = [0;450;900];

% deflection data was measured according to the experimental apparatus in
% the paper
deflections = zeros(5,3,4);
deflections(:,:,1) = [[0, 8, 17]; [0, 6, 11]; [0, 5, 10]; [0, 3, 6]; [0, 0.25, 1]];

deflections(:,2:3,2) = [2.4	4.3
1.6	2.15
1	1.7
0.4	0.7
0.1	0.1];

deflections(:,2:3,3) = [2.2,	3;
                1.45, 2.35;
                1.1,	1.8;
                0.6,	1;
                0.2,	0.45];

deflections(:,2:3,4) = [0.8	1.7
0.55	1.1
0.4	1
0.2	0.7
0.1	0.1];

plot(lengths, deflections(:,3), lengths, deflections(:,2))

% stiffnesses = [(weights(2)/1000*9.81)./(deflections(:,2)./1000),(weights(3)/1000*9.81)./(deflections(:,3)./1000)]
stiffnesses = squeeze([(weights(3)/1000*9.81)./(deflections(:,3,:)./1000)])

semilogy(lengths(1:end)/1000, stiffnesses(1:end,2:end), 'LineWidth', 2)
% hold on 
% semilogy(lengths(1:end)/1000, stiffnesses(1:end,2), 'LineWidth', 2)

xlabel("$c$ (m)", 'Interpreter', 'latex', 'FontSize', 28)
ylabel("Stiffness (N/m)", 'Interpreter', 'latex', 'FontSize', 28)
set(gca,'FontSize',14)
h = gcf;

ax = gca;
% Set the font size
set(ax, 'FontSize', 24)
% Set the interpreter for the tick labels
set(ax, 'TickLabelInterpreter', 'latex')

% Modify the figure size
set(gcf, 'Position', [303 495 560 660])


% Get the legend and set its interpreter
% l = findobj(gcf, 'Type', 'legend');
% set(l, 'Interpreter', 'latex')

% Turn on grid and box
grid on
box on 
fig = gcf;
% if save_fig == 1
%     print(fig, "cartesian_end_effector", '-dpdf', '-vector')
% end
