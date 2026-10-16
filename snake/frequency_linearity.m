% frequency_linearity.m -- is displacement proportional to frequency for the best gaits?
% Runs the 4 best gaits from joint_sweep.m at 8 frequencies between 1 and 2.65 Hz
% (32 runs, about 10 min). Run from the MAT-DiSMech root folder.

clc; close all;
addpath springs/ util_functions/ contact_functions/ rod_dynamics/ shell_dynamics/ external_forces/ adaptive_stepping/ logging/
addpath(genpath('experiments'));

L_body = 0.15; T_meas = 10;                       % body length (m), measurement time (s)
names = {'Best land gait / land', 'Best land gait / water', ...
    'Best aquatic gait / land', 'Best aquatic gait / water'};
freqs = linspace(1.0, 2.65, 8);

D = nan(4, numel(freqs));                         % forward displacement in 10 s (m)
C = nan(4, numel(freqs));                         % steady distance per cycle (m), second half of the 10 s
figure; tl = tiledlayout(1, 2);
ax1 = nexttile; hold(ax1, 'on'); ax2 = nexttile; hold(ax2, 'on');

for c = 1:4
    S = load(sprintf('joint_sweep_case%d.mat', c), 'A_opt', 'L_opt', 'loco', 'envt');
    for j = 1:numel(freqs)
        try
            [D(c,j), ~, ~, ~, ~, C(c,j)] = custom_main_snake(S.A_opt, freqs(j), S.L_opt, pi/2, S.loco, S.envt, struct());
        catch err
            fprintf('Run failed (%s, %.2f Hz): %s\n', names{c}, freqs(j), err.message);
        end
    end
    ok = ~isnan(D(c,:));
    f = freqs(ok); d = D(c,ok);
    k = sum(f.*d)/sum(f.^2);                      % line through the origin: D = k f
    fit = k*f;
    R2 = 1 - sum((d - fit).^2)/sum((d - mean(d)).^2);
    dev = 100*max(abs(d - fit)./d);               % worst point vs the line (%)
    p = polyfit(f, d, 1);                         % free line: D = p(1) f + p(2)
    stride = d./(T_meas*f)/L_body;                % distance per cycle (body lengths)
    fprintf('%-26s (A=%.4f, lambda=%.2f), %d of %d runs valid\n', names{c}, S.A_opt, S.L_opt, nnz(ok), numel(freqs));
    fprintf('   stride %.3f to %.3f L/cycle (range %.1f %% of mean)\n', min(stride), max(stride), 100*(max(stride)-min(stride))/mean(stride));
    fprintf('   fit through origin: R^2 = %.5f, worst point %.1f %% off the line\n', R2, dev);
    fprintf('   free fit intercept = %+.1f %% of the mean displacement\n', 100*p(2)/mean(d));
    h = plot(ax1, f, d, 'o', 'DisplayName', names{c});
    plot(ax1, [0 max(f)], k*[0 max(f)], '--', 'Color', h.Color, 'HandleVisibility', 'off');
    plot(ax2, f, stride, 'o-', 'DisplayName', names{c});
end
xlabel(ax1, 'Frequency (Hz)'); ylabel(ax1, 'Forward displacement in 10 s (m)');
title(ax1, 'Dashed: D = k f'); legend(ax1, 'Location', 'northwest'); grid(ax1, 'on'); xlim(ax1, [0 inf]);
xlabel(ax2, 'Frequency (Hz)'); ylabel(ax2, 'Distance per cycle (body lengths)');
title(ax2, 'Flat line = no frequency effect'); legend(ax2, 'Location', 'best'); grid(ax2, 'on'); ylim(ax2, [0 inf]);
save('frequency_linearity.mat', 'freqs', 'D', 'C', 'names');