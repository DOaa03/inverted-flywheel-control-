function draw42(y, l, r, figTitle, u, dt)


[N, ncols] = size(y);
t = (0:N-1)' * dt;

theta_p = y(:, 1); 
theta_f = y(:, 2);  
dot_theta_p = y(:, 3);
dot_theta_f = y(:, 4); 

%flywheel center 
x2 = l * sin(theta_p);
y2 = l * cos(theta_p);

fig = figure('Name', figTitle, 'NumberTitle', 'off');
set(fig, 'Color', 'w');


xlim_anim = [-l - r , l + r ];
ymin_anim = -l - r ; 
ymax_anim = l + r ; 


% subplot 1: Animation
ax1 = subplot(2,2,1);
hold(ax1,'on');
axis(ax1, [xlim_anim ymin_anim ymax_anim]); axis(ax1,'equal');
grid(ax1,'on');
title(ax1, 'Inverted Flywheel Animation');
xlabel(ax1,'X (m)');
ylabel(ax1,'Y (m)');

h_flywheel_lines = gobjects(4, 1);
for i = 1:4
    h_flywheel_lines(i) = plot(ax1, [0, 0], [0, 0], 'k-', 'LineWidth', 1.5, 'Color', '#7e2c7f');
end
pendLine = line(ax1, [0 x2(1)], [0 y2(1)], 'LineWidth', 3, 'Color', '#0e3579');

flywheel = rectangle(ax1, 'Position', [x2(1) - r, y2(1) - r, 2*r, 2*r], ...
    'Curvature', [1, 1],'FaceColor', 'none', 'EdgeColor', '#7e2c7f', 'LineWidth', 2);


% subplot 2: Control Input
ax2 = subplot(2,2,2);
h_u = plot(ax2, t(1), u(1), 'LineWidth', 1.5, 'Color', '#0072BD');
grid(ax2,'on');
xlabel(ax2,'Time (s)');
ylabel(ax2,'Torque Input (u)');
title(ax2,'Control Input ($\tau$)', 'Interpreter', 'latex');

xlim(ax2,[t(1) t(end)]);
u_min = min(u); u_max = max(u);
ylim(ax2, [u_min - 0.1, u_max + 0.1]);


% subplot 3: Pendulum Angle 
ax3 = subplot(2,2,3);
h_theta_p = plot(ax3, t(1), theta_p(1), 'LineWidth', 1.5, 'DisplayName', 'Angle $\theta_p$', 'Color', '#D95319');
hold on;
refernce=plot(ax3,[t(1) t(end)], [0 0], 'k--','LineWidth',1, 'DisplayName','refernce Angle $\theta_p$'); 
grid(ax3,'on');
xlabel(ax3,'Time (s)');
ylabel(ax3,'$\theta_p$ (rad/s)', 'Interpreter', 'latex');
title(ax3,'Pendulum Angle');
legend(ax3, 'show', 'Interpreter', 'latex', 'Location', 'best');
xlim(ax3,[t(1) t(end)]);
p_min = min(theta_p); p_max = max(theta_p);
ylim(ax3, [p_min , p_max ]);


% subplot 4: Flyhweel  Angular Velocity
ax4 = subplot(2,2,4);
h_dot_theta_f = plot(ax4, t(1), dot_theta_f(1), 'LineWidth', 1.5, 'LineStyle', '--', 'DisplayName', 'Ang Vel $\dot{\theta}_f$', 'Color', '#4DBEEF');
hold on;
refernce=plot(ax4,[t(1) t(end)], [0 0], 'k--','DisplayName','refernce Ang Vel $\dot{\theta}_f$'); 
grid(ax4,'on');
xlabel(ax4,'Time (s)');
ylabel(ax4,' $\dot{\theta}_f$ ( rad/s)', 'Interpreter', 'latex');
title(ax4,'Flywheel Angular Velocity');
legend(ax4, 'show', 'Interpreter', 'latex', 'Location', 'best');
xlim(ax4,[t(1) t(end)]);
f_min = min( dot_theta_f); f_max = max( dot_theta_f);
ylim(ax4, [f_min , f_max ]);



pause(2);
for k = 1:N
    
    set(flywheel, 'Position', [x2(k) - r, y2(k) - r, 2*r, 2*r]);
    set(pendLine, 'XData', [0 x2(k)], 'YData', [0 y2(k)]);
    
    angle_absolute = theta_p(k) + theta_f(k);
    
    angles = [angle_absolute, angle_absolute + pi/2, angle_absolute + pi, angle_absolute + 3*pi/2];
    
    for i = 1:4
        x_line_end = x2(k) + r * sin(angles(i));
        y_line_end = y2(k) + r * cos(angles(i));
        
        set(h_flywheel_lines(i), 'XData', [x2(k), x_line_end], 'YData', [y2(k), y_line_end]);
    end
    
    set(h_u, 'XData', t(1:k), 'YData', u(1:k));
    set(h_theta_p, 'XData', t(1:k), 'YData', theta_p(1:k));
    set(h_dot_theta_f, 'XData', t(1:k), 'YData', dot_theta_f(1:k));
    
    drawnow;
    pause(dt);
    
end

end