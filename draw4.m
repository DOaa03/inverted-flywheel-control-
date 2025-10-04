function draw4(y,l, r, figTitle, u, dt)

[N, ncols] = size(y);
t = (0:N-1)' * dt;
x2=l*sin(y(:,1));
y2=l*cos(y(:,1));
theta = y(:,3);             % pendulum angle (rad)


fig = figure('Name', figTitle, 'NumberTitle', 'off');
set(fig, 'Color', 'w');

margin = 0.5;
xlim_anim = [min(x)-2-margin, max(x)+2+margin];
ymin_anim = -0.5; ymax_anim = 1.5 + L + 0.5;

% subplot 1: animation
ax1 = subplot(2,2,1);
hold(ax1,'on');
axis(ax1, [xlim_anim ymin_anim ymax_anim]);
axis(ax1,'equal');
grid(ax1,'on');
title(ax1, 'Animation');



flywheel = rectangle(ax1, 'Position', [x2(1) - r, y2(1) - r, 2*r, 2*r], ...
    'Curvature', [1, 1],'FaceColor', '#7e2c7f', 'EdgeColor', '#7e2c7f');
pendLine = line(ax1, [0 x2(1)],[0 y2(1)], 'LineWidth', 2, 'Color', '#0e3579');
refernce=line(ax1,[0, 0],[ymin_anim ,ymax_anim],'Color', 'r' );
% subplot 2: control input
ax2 = subplot(2,2,2);
h_u = plot(ax2, t(1), u(1), 'LineWidth', 1.5);
grid(ax2,'on');
xlabel(ax2,'Time (s)');
ylabel(ax2,'u');
title(ax2,'Control Input');
xlim(ax2,[t(1) t(end)]);
u_min = min(u); u_max = max(u);
ylim(ax2, [u_min-0.1*(abs(u_min)+1), u_max+0.1*(abs(u_max)+1)]);

% subplot 3: cart position
ax3 = subplot(2,2,3);
h_x = plot(ax3, t(1), x(1), 'LineWidth', 1.5);
hold on
refernce=plot(ax3,0:length(t), zeros(1,length(t)+1));
grid(ax3,'on');
xlabel(ax3,'Time (s)');
ylabel(ax3,'x (m)');
title(ax3,'Cart Position');
xlim(ax3,[t(1) t(end)]);
x_min = min(x); x_max = max(x);
ylim(ax3, [x_min-0.2*abs(x_min)-0.5, x_max+0.2*abs(x_max)+0.5]);

% subplot 4: pendulum angle
ax4 = subplot(2,2,4);
h_theta = plot(ax4, t(1), theta(1), 'LineWidth', 1.5);
hold on
refernce=plot(ax4,0:length(t), zeros(1,length(t)+1));
grid(ax4,'on');
xlabel(ax4,'Time (s)');
ylabel(ax4,'\theta (rad)');
title(ax4,'Pendulum Angle');
xlim(ax4,[t(1) t(end)]);
th_min = min(theta); th_max = max(theta);
ylim(ax4, [th_min-0.2*abs(th_min)-0.2, th_max+0.2*abs(th_max)+0.2]);

pause(2);
for k = 1:N
    

    set(flywheel, 'Position', [x2(k) - r, y2(k) - r, 2*r, 2*r]);
    
    set(pendLine, 'XData', [0 x2(k)], 'YData', [0 y2(k));
    
    set(h_u, 'XData', t(1:k), 'YData', u(1:k));
    set(h_x, 'XData', t(1:k), 'YData', x(1:k));
    set(h_theta, 'XData', t(1:k), 'YData', theta(1:k));
    

    x_line_end = x2(k) + r * cos(y(k,2));
    y_line_end = y2(k) + r * sin(y(k,2));
    x_line_end2 = x2(k) + r * cos(y(k,2)+pi);
    y_line_end2 = y2(k) + r * sin(y(k,2)+pi);
    x_line_end3 = x2(k)+ r * cos(y(k,2)+pi/2);
    y_line_end3 = y2(k)+ r * sin(y(k,2)+pi/2);
    x_line_end4 = x2(k)+ r * cos(y(k,2)+3*pi/2);
    y_line_end4 = y2(k) + r * sin(y(k,2)+3*pi/2);
    plot([x2(k), x_line_end], [y2(k), y_line_end], 'k-', 'LineWidth', 2);
    plot([x2(k), x_line_end2], [y2(k), y_line_end2], 'k-', 'LineWidth', 2);
    plot([x2(k), x_line_end3], [y2(k), y_line_end3], 'k-', 'LineWidth', 2);
    plot([x2(k), x_line_end4], [y2(k), y_line_end4], 'k-', 'LineWidth', 2);
    drawnow;
    pause(dt);
    
    
end

end
