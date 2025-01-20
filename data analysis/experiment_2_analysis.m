% Define the relative error list
rel1 = [%list of relative errors from all points in all trials];
rel = rel1*100;%percentage error

% Calculate mean and standard deviation
mean_rel = mean(rel);
std_rel = std(rel);

% Generate the range for the normal distribution curve
x = linspace(mean_rel - 3*std_rel, mean_rel + 3*std_rel, 1000);
pdf_values = (1 / (std_rel * sqrt(2 * pi))) * exp(-0.5 * ((x - mean_rel) / std_rel).^2);

% Plot the normal distribution
figure;
plot(x, pdf_values, 'LineWidth', 2);
hold on;

% Add vertical lines for mean, mean ± 1 SD, and mean ± 2 SD
xline(mean_rel, 'r', 'LineWidth', 1.5, 'Label', 'Mean');
xline(mean_rel + std_rel, 'g--', 'LineWidth', 1.5, 'Label', 'Mean + 1 SD');
xline(mean_rel - std_rel, 'g--', 'LineWidth', 1.5, 'Label', 'Mean - 1 SD');
xline(mean_rel + 2*std_rel, 'b:', 'LineWidth', 1.5, 'Label', 'Mean + 2 SD');
xline(mean_rel - 2*std_rel, 'b:', 'LineWidth', 1.5, 'Label', 'Mean - 2 SD');

% Annotate the graph with mean and standard deviation values
text(mean_rel, max(pdf_values) * 0.8, sprintf('Mean: %.4f', mean_rel), 'Color', 'r', 'FontSize', 12);
text(mean_rel + std_rel, max(pdf_values) * 0.6, sprintf('SD: %.4f', std_rel), 'Color', 'g', 'FontSize', 12);

% Add titles and labels
title('Normal Distribution of Percentage Error at 4 points');
xlabel('Percentage Error');
ylabel('Probability Density');
grid on;
hold off;

