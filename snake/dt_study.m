% dt_study.m -- time step sensitivity for the four best gaits (reviewer comments MF2, MF8)
% For each best gait it runs:
%   base   : 100 steps per cycle (the normal setting)
%   half   : 200 steps per cycle (half the time step)
%   double :  50 steps per cycle (double the time step)
% All runs use Newton tolerance 1e-6 and exact_dt (no 7 ms cap), so the time step really
% is halved and doubled. Needs the exact_dt option in custom_main_snake.m.
% Run this script from the MAT-DiSMech root folder.

clc; close all;
addpath springs/ util_functions/ contact_functions/ rod_dynamics/ shell_dynamics/ external_forces/ adaptive_stepping/ logging/
addpath(genpath('experiments'));

freq = 1.825;
case_names = {'Best land gait / land', 'Best land gait / water', ...
              'Best aquatic gait / land', 'Best aquatic gait / water'};

% points: [A21, wavelength, frequency, loco_type, env_type]
points = [0.3602  1.2  freq 1 1;     % land gait, land
          0.2041  2.0  freq 1 2;     % land gait, water
          0.4122  1.2  freq 2 1;     % aquatic gait, land
          0.2041  2.0  freq 2 2];    % aquatic gait, water

variants = {struct('steps_per_cycle', 100, 'tol', 1e-6, 'exact_dt', true), ...   % base
            struct('steps_per_cycle', 200, 'tol', 1e-6, 'exact_dt', true), ...   % half dt
            struct('steps_per_cycle',  50, 'tol', 1e-6, 'exact_dt', true)};      % double dt

R = nan(size(points, 1), 3);
for k = 1:size(points, 1)
    p = points(k, :);
    for v = 1:3
        try
            R(k, v) = custom_main_snake(p(1), p(3), p(2), pi/2, p(4), p(5), variants{v});
        catch err
            fprintf('Run failed (%s, variant %d): %s\n', case_names{k}, v, err.message);
        end
    end
    fprintf('%-26s A=%.4f lambda=%.3f: base %.4f m | dt/2 %+.2f %% | 2dt %+.2f %%\n', ...
        case_names{k}, p(1), p(2), R(k,1), ...
        100*(R(k,2) - R(k,1))/R(k,1), 100*(R(k,3) - R(k,1))/R(k,1));
end
save('dt_study.mat', 'points', 'case_names', 'R');