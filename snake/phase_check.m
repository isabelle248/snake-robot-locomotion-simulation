% phase_check.m -- does a constant phase offset change the displacement?
clc; close all;
addpath springs/ util_functions/ contact_functions/ rod_dynamics/ shell_dynamics/ external_forces/ adaptive_stepping/ logging/
addpath(genpath('experiments'));

% [A21, wavelength, frequency, loco_type, env_type]
best = [0.3602 1.2 1.825 1 1; 0.2041 2.0 1.825 1 2; 0.4122 1.2 1.825 2 1; 0.2041 2.0 1.825 2 2];

names = {'Best land gait / land', 'Best land gait / water', 'Best aquatic gait / land', 'Best aquatic gait / water'};
phases = (0:7)*pi/4;                        % 0, pi/4, ..., 7pi/4
j_ref = find(abs(phases - pi/2) < 1e-9);    % pi/2 is the phase used in the main runs

D = nan(4, numel(phases));
figure; hold on;
for c = 1:4
    p = best(c, :);
    for j = 1:numel(phases)
        try
            D(c,j) = custom_main_snake(p(1), p(3), p(2), phases(j), p(4), p(5), struct());
        catch err
            fprintf('Run failed (%s, phase %.2f): %s\n', names{c}, phases(j), err.message);
        end
    end
    chg = 100*(D(c,:) - D(c,j_ref))./D(c,j_ref);          % change relative to phase = pi/2
    spread = 100*(max(D(c,:)) - min(D(c,:)))/mean(D(c,:), 'omitnan');
    fprintf('%-26s D at pi/2 = %.4f m | range over phases %.3f %% of mean | largest change vs pi/2 %.3f %%\n', ...
        names{c}, D(c,j_ref), spread, max(abs(chg)));
    plot(phases, chg, 'o-', 'DisplayName', names{c});
end
xlabel('Phase offset (rad)'); ylabel('Change in displacement vs phase = \pi/2 (%)');
title('Effect of a constant phase offset'); legend('Location', 'best'); grid on;
save('phase_check.mat', 'best', 'phases', 'D', 'names');