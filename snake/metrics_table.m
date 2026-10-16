
% metrics_table.m
% Run from the MAT-DiSMech main folder. Reruns the four best gaits and
% saves all outputs of custom_main_snake to metrics_table.csv. About 45 seconds.
% Uses A_opt, L_opt from joint_sweep_case1..4.mat if they are in the current folder;
% otherwise uses the values from the report.

clear; clc;
f = 1.825;  phase = pi/2;  L = 0.15;
opts = struct('verbose', false);

% case_id: 1 = land gait/land, 2 = land gait/water, 3 = aquatic gait/land, 4 = aquatic gait/water
cases = [1 1; 1 2; 2 1; 2 2];           % [loco_type env_type]
names = ["Land gait / land", "Land gait / water", "Aquatic gait / land", "Aquatic gait / water"];
A_default = [0.3602 0.2041 0.4122 0.2041];  % from the report (Section 3)
W_default = [1.20  2.00  1.20  2.00];

rows = {};
for c = 1:4
    A = A_default(c);  W = W_default(c);
    rows(end+1,:) = run_one(names(c), A, W, f, phase, cases(c,1), cases(c,2), opts, L); %#ok<SAGROW>
end

T = cell2table(rows, 'VariableNames', {'case','amp','wavelength','forward_disp_m', ...
    'lateral_drift_m','travel_angle_deg','yaw_deg','steady_speed_ms','steady_speed_BLs', ...
    'distance_per_cycle_m','distance_per_cycle_BL','min_gap_mm'});
disp(T);
writetable(T, 'metrics_table.csv');
fprintf('Saved metrics_table.csv\n');

function r = run_one(name, A, W, f, phase, loco, envt, opts, L)
[fwd, ~, lat, yaw, U, dpc, info] = custom_main_snake(A, f, W, phase, loco, envt, opts);
r = {char(name), A, W, fwd, lat, info.travel_angle_deg, yaw*180/pi, U, U/L, dpc, dpc/L, 1e3*info.min_gap};
end

