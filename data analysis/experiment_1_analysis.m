% Parameters for robot arm
v1_current = [% list of current motor angle 1 in multiple trials];
v2_current = [% list of current motor angle 1 in multiple trials];
self.position_input_x.get = %desired x coordinate;
self.position_input_y.get = %desired y coordinate;
desired_x = self.position_input_x.get;
desired_y = self.position_input_y.get;

r1 = 120.0;    % Length of first link
r2 = 92.0;     % Length of second link

abs_err_list = [];
rel_err_list = [];
per_err_list = [];

for i = 1:numel(v1_current)
    t1 = v1_current(i);    % Current angle of joint 1
    t2 = v2_current(i);    % Current angle of joint 2
    T = forward_kinematics(r1, r2, t1, t2);
    current_x = T(1, 4);
    current_y = T(2, 4);

    abs_err = sqrt((desired_x - current_x)^2 + (desired_y - current_y)^2);
    rel_err = abs_err / sqrt((desired_x)^2 + (desired_y)^2);
    per_err = rel_err*100;
    abs_err_list = [abs_err_list, abs_err];
    rel_err_list = [rel_err_list, rel_err];
    per_err_list = [per_err_list, per_err];
end

mean_err = mean(abs_err_list);
std_err = std(abs_err_list);

% Plotting the normal distribution based on the calculated mean and standard deviation
x = mean_err - 3*std_err:0.01:mean_err + 3*std_err; % Range of x values
pdf_values = normpdf(x, mean_err, std_err);

% Plot the normal distribution
figure;
plot(x, pdf_values, 'LineWidth', 2);
hold on;

% Add vertical lines for mean error, mean ± 1 SD, and mean ± 2 SD
line([mean_err mean_err], [0 max(pdf_values)], 'Color', 'r', 'LineStyle', '-', 'LineWidth', 1.5, 'DisplayName', 'Mean Error');
line([mean_err - std_err mean_err - std_err], [0 max(pdf_values)], 'Color', 'g', 'LineStyle', '--', 'LineWidth', 1.5, 'DisplayName', 'Mean - 1 SD');
line([mean_err + std_err mean_err + std_err], [0 max(pdf_values)], 'Color', 'g', 'LineStyle', '--', 'LineWidth', 1.5, 'DisplayName', 'Mean + 1 SD');
line([mean_err - 2*std_err mean_err - 2*std_err], [0 max(pdf_values)], 'Color', 'b', 'LineStyle', ':', 'LineWidth', 1.5, 'DisplayName', 'Mean - 2 SD');
line([mean_err + 2*std_err mean_err + 2*std_err], [0 max(pdf_values)], 'Color', 'b', 'LineStyle', ':', 'LineWidth', 1.5, 'DisplayName', 'Mean + 2 SD');

% Annotate the graph with values
text(mean_err, max(pdf_values) * 0.8, sprintf('Mean: %.4f', mean_err), 'Color', 'r', 'FontSize', 12);
text(mean_err + std_err, max(pdf_values) * 0.6, sprintf('SD: %.4f', std_err), 'Color', 'g', 'FontSize', 12);

title(['Normal Distribution Curve of absolute error at (', num2str(desired_x), ', ', num2str(desired_y), ')']);
xlabel('Distance (mm)');
ylabel('Probability Density');
legend show;
grid on;
hold off;

mean_err_per = mean(per_err_list);
std_err_per = std(per_err_list);

% Plotting the normal distribution based on the calculated mean and standard deviation
x = mean_err_per - 3*std_err_per:0.01:mean_err_per + 3*std_err_per; % Range of x values
pdf_value = normpdf(x, mean_err_per, std_err_per);

% Plot the normal distribution
figure;
plot(x, pdf_value, 'LineWidth', 2);
hold on;

% Add vertical lines for mean error, mean ± 1 SD, and mean ± 2 SD
line([mean_err_per mean_err_per], [0 max(pdf_value)], 'Color', 'r', 'LineStyle', '-', 'LineWidth', 1.5, 'DisplayName', 'Mean Error');
line([mean_err_per - std_err_per mean_err_per - std_err_per], [0 max(pdf_value)], 'Color', 'g', 'LineStyle', '--', 'LineWidth', 1.5, 'DisplayName', 'Mean - 1 SD');
line([mean_err_per + std_err_per mean_err_per + std_err_per], [0 max(pdf_value)], 'Color', 'g', 'LineStyle', '--', 'LineWidth', 1.5, 'DisplayName', 'Mean + 1 SD');
line([mean_err_per - 2*std_err_per mean_err_per - 2*std_err_per], [0 max(pdf_value)], 'Color', 'b', 'LineStyle', ':', 'LineWidth', 1.5, 'DisplayName', 'Mean - 2 SD');
line([mean_err_per + 2*std_err_per mean_err_per + 2*std_err_per], [0 max(pdf_value)], 'Color', 'b', 'LineStyle', ':', 'LineWidth', 1.5, 'DisplayName', 'Mean + 2 SD');

% Annotate the graph with values
text(mean_err_per, max(pdf_value) * 0.8, sprintf('Mean: %.4f', mean_err_per), 'Color', 'r', 'FontSize', 12);
text(mean_err_per + std_err_per, max(pdf_value) * 0.6, sprintf('SD: %.4f', std_err_per), 'Color', 'g', 'FontSize', 12);

title(['Normal Distribution Curve of percentage error at (', num2str(desired_x), ', ', num2str(desired_y), ')']);
xlabel('Percentage error (%)');
ylabel('Probability Density');
legend show;
grid on;
hold off;




rel_err_list
abs_err_list
per_err_list