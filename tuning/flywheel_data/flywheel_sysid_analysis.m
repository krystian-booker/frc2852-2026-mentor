%% flywheel_sysid_analysis.m
% Flywheel System Identification & PID Auto-Tuning for CTRE Phoenix6
%
% Loads CSV data from FlywheelSysIdCommand and computes feedforward and PID
% gains ready to paste into Constants.java FlywheelConstants.
%
% All outputs are in CTRE VelocityTorqueCurrentFOC units:
%   kS: Amps                 (static friction feedforward)
%   kV: Amps / RPS           (velocity feedforward)
%   kA: Amps / (RPS/s)       (acceleration feedforward)
%   kP: Amps / RPS           (proportional gain)
%   kI: Amps / rotation      (integral gain)
%   kD: Amps / (RPS/s)       (derivative gain)
%
% Requires: System Identification Toolbox, Control System Toolbox
%
% CSV phase IDs:
%   0 = Quasistatic voltage ramp (0-10V)
%   1 = Voltage step tests (2V, 4V, 6V, 8V, 10V)
%   2 = Coast-down (spinup then free deceleration)
%   3 = Current step tests (5A, 10A, 15A, 20A, 30A)

clear; clc; close all;

%% ===================== USER CONFIGURATION =====================
% Adjust these to tune the analysis and PID design

% Quasistatic filtering
QS_MIN_VELOCITY_RPS   = 0.5;   % Ignore data below this speed (stiction noise)
QS_ACCEL_THRESHOLD    = 1.0;   % RPS/s - max accel to count as steady-state
QS_SMOOTH_WINDOW      = 11;    % Moving average window for derivative smoothing

% System identification
SYSID_MODEL_ORDER     = 1;     % 1 = first-order, 2 = second-order (1 recommended)

% PID tuning targets
PID_BANDWIDTH         = 15;    % rad/s - closed-loop bandwidth target
PID_PHASE_MARGIN      = 60;    % degrees - phase margin target

% Existing gains (for comparison)
EXISTING_KS = 2.1000;
EXISTING_KV = 0.2203;
EXISTING_KA = 0.6749;
EXISTING_KP = 8.0000;
EXISTING_KI = 0.0000;
EXISTING_KD = 0.1600;

%% ===================== LOAD DATA =====================
[filename, pathname] = uigetfile('*.csv', 'Select flywheel SysId CSV file');
if isequal(filename, 0)
    error('No file selected.');
end
data = readtable(fullfile(pathname, filename));

% Parse columns
t_raw       = data.timestamp;
phase       = data.test_phase;
cmd_type    = data.command_type;
cmd_value   = data.command_value;
vel_rps     = data.velocity_rps;
vel_rpm     = data.velocity_rpm;
motor_v     = data.motor_voltage;
stator_i    = data.stator_current;
supply_v    = data.supply_voltage;

% Zero-base time
t = t_raw - t_raw(1);

% Compute actual sample period from data
dt_vec = diff(t);
Ts = median(dt_vec);

fprintf('Loaded: %s\n', filename);
fprintf('  %d samples, %.1f seconds, Ts = %.4f s (%.0f Hz)\n', ...
    height(data), t(end), Ts, 1/Ts);
fprintf('  Supply voltage: %.1f - %.1f V (mean %.1f V)\n', ...
    min(supply_v), max(supply_v), mean(supply_v));

%% ===================== SEPARATE PHASES =====================
idx_qs = phase == 0;   % Quasistatic ramp
idx_vs = phase == 1;   % Voltage steps
idx_cd = phase == 2;   % Coast-down
idx_cs = phase == 3;   % Current steps

fprintf('  Phase samples: QS=%d  VSteps=%d  Coast=%d  CSteps=%d\n\n', ...
    sum(idx_qs), sum(idx_vs), sum(idx_cd), sum(idx_cs));

%% ===================== RAW DATA OVERVIEW =====================
figure('Name', 'Raw Test Data', 'Position', [50 50 1400 800]);

ax1 = subplot(4,1,1);
plot(t, vel_rps, 'b', 'LineWidth', 0.8);
ylabel('Velocity (RPS)');
title('Flywheel System Identification - Raw Data');
grid on;

ax2 = subplot(4,1,2);
hold on;
plot(t(cmd_type==0), cmd_value(cmd_type==0), 'r.', 'MarkerSize', 3);
plot(t(cmd_type==1), cmd_value(cmd_type==1), 'b.', 'MarkerSize', 3);
ylabel('Command');
legend('Voltage (V)', 'Current (A)', 'Location', 'best');
grid on;

ax3 = subplot(4,1,3);
plot(t, stator_i, 'Color', [0.85 0.33 0.1], 'LineWidth', 0.8);
ylabel('Stator Current (A)');
grid on;

ax4 = subplot(4,1,4);
plot(t, motor_v, 'm', 'LineWidth', 0.8);
ylabel('Motor Voltage (V)');
xlabel('Time (s)');
grid on;

linkaxes([ax1 ax2 ax3 ax4], 'x');

%% ===================== FEEDFORWARD: QUASISTATIC RAMP =====================
% During the voltage ramp, the motor draws current to maintain velocity.
% At true steady-state (near-zero acceleration): stator_current = kS + kV * velocity
% We filter for low-acceleration points to avoid multicollinearity between
% velocity and acceleration (they're correlated during a ramp).

fprintf('=== FEEDFORWARD (Quasistatic Ramp) ===\n');

qs_t   = t(idx_qs);
qs_vel = vel_rps(idx_qs);
qs_i   = stator_i(idx_qs);

% Smooth velocity and compute acceleration
qs_vel_smooth = movmean(qs_vel, QS_SMOOTH_WINDOW);
qs_accel = gradient(qs_vel_smooth, Ts);

% Filter: positive velocity AND low acceleration (true steady-state only)
valid = qs_vel_smooth > QS_MIN_VELOCITY_RPS & abs(qs_accel) < QS_ACCEL_THRESHOLD;

qs_vel_f = qs_vel_smooth(valid);
qs_i_f   = qs_i(valid);
n_qs     = sum(valid);

if n_qs < 20
    % Relax threshold if too few points pass the filter
    fprintf('  WARNING: Only %d points below %.1f RPS/s accel threshold.\n', n_qs, QS_ACCEL_THRESHOLD);
    fprintf('  Relaxing to 2.0 RPS/s...\n');
    valid = qs_vel_smooth > QS_MIN_VELOCITY_RPS & abs(qs_accel) < 2.0;
    qs_vel_f = qs_vel_smooth(valid);
    qs_i_f   = qs_i(valid);
    n_qs     = sum(valid);
end

% Simple 2-parameter linear regression: I = kS + kV * vel
% This avoids multicollinearity (velocity & acceleration are correlated in ramp)
X_qs = [ones(n_qs, 1), qs_vel_f];
ff_coeffs = X_qs \ qs_i_f;

kS_qs = ff_coeffs(1);   % Amps
kV_qs = ff_coeffs(2);   % Amps / RPS

% R-squared
qs_i_pred = X_qs * ff_coeffs;
SS_res = sum((qs_i_f - qs_i_pred).^2);
SS_tot = sum((qs_i_f - mean(qs_i_f)).^2);
R2_qs = 1 - SS_res / SS_tot;

fprintf('  Steady-state fit: I = %.4f + %.4f * vel  (R^2 = %.4f)\n', ...
    kS_qs, kV_qs, R2_qs);
fprintf('  kS = %.4f A\n', kS_qs);
fprintf('  kV = %.4f A/RPS\n', kV_qs);
fprintf('  Data points used: %d / %d (accel < %.1f RPS/s)\n\n', ...
    n_qs, sum(idx_qs), QS_ACCEL_THRESHOLD);

% Plot
figure('Name', 'Quasistatic Feedforward', 'Position', [50 50 1100 700]);

subplot(2,2,1);
plot(qs_t, qs_vel, 'b', 'LineWidth', 0.8);
hold on;
plot(qs_t(valid), qs_vel_f, 'r.', 'MarkerSize', 4);
ylabel('Velocity (RPS)');
xlabel('Time (s)');
legend('All data', 'Steady-state points');
title(sprintf('Quasistatic Ramp (|accel| < %.1f RPS/s filter)', QS_ACCEL_THRESHOLD));
grid on;

subplot(2,2,2);
scatter(qs_vel_f, qs_i_f, 8, 'b', 'filled', 'MarkerFaceAlpha', 0.3);
hold on;
vel_range = linspace(0, max(qs_vel_f)*1.1, 100);
plot(vel_range, kS_qs + kV_qs * vel_range, 'r-', 'LineWidth', 2);
xlabel('Velocity (RPS)');
ylabel('Stator Current (A)');
title(sprintf('I = %.3f + %.4f * \\omega  (R^2 = %.4f)', kS_qs, kV_qs, R2_qs));
legend('Measured', 'Linear fit');
grid on;

subplot(2,2,3);
plot(qs_t(valid), qs_i_f, 'b', 'LineWidth', 0.8);
hold on;
plot(qs_t(valid), qs_i_pred, 'r', 'LineWidth', 1.2);
ylabel('Stator Current (A)');
xlabel('Time (s)');
legend('Measured', 'Predicted (kS + kV*v)');
title(sprintf('Regression Fit (R^2 = %.4f)', R2_qs));
grid on;

subplot(2,2,4);
plot(qs_t(valid), qs_i_f - qs_i_pred, 'Color', [0.5 0.5 0.5]);
ylabel('Residual (A)');
xlabel('Time (s)');
title('Regression Residuals');
grid on;

%% ===================== SYSTEM ID: CURRENT STEPS =====================
% Identify transfer function: current (A) -> velocity (RPS)
% G(s) = K / (tau*s + 1) for first-order
%
% IMPORTANT: We only use the "hold" portions where current is actively
% commanded (cmd_type == 1). The settle periods use NeutralOut (coast),
% which is NOT the same as commanding 0A — coast disconnects the motor
% drive entirely. Including those periods would confuse the model into
% estimating an artificially long time constant.

fprintf('=== SYSTEM IDENTIFICATION (Current Steps) ===\n');

cs_t   = t(idx_cs);
cs_vel = vel_rps(idx_cs);
cs_cmd = cmd_value(idx_cs);
cs_ct  = cmd_type(idx_cs);

% Segment into individual step experiments (hold portions only)
% Each segment where cmd_type == 1 is one experiment
hold_mask = cs_ct == 1;
segments = diff([0; hold_mask; 0]);
seg_starts = find(segments == 1);
seg_ends   = find(segments == -1) - 1;
n_segments = length(seg_starts);

fprintf('  Found %d current step segments\n', n_segments);

% Build multi-experiment iddata by merging individual step segments
experiments = {};
for k = 1:n_segments
    seg_idx = seg_starts(k):seg_ends(k);
    if length(seg_idx) < 10
        continue;  % Skip very short segments
    end

    seg_t   = cs_t(seg_idx);
    seg_vel = cs_vel(seg_idx);
    seg_cmd = cs_cmd(seg_idx);

    % Resample segment onto uniform time grid
    seg_t_rel = seg_t - seg_t(1);
    seg_t_uni = (0 : Ts : seg_t_rel(end))';
    if length(seg_t_uni) < 10
        continue;
    end
    seg_vel_uni = interp1(seg_t_rel, seg_vel, seg_t_uni, 'linear', 'extrap');
    seg_cmd_uni = interp1(seg_t_rel, seg_cmd, seg_t_uni, 'previous', 'extrap');

    seg_data = iddata(seg_vel_uni, seg_cmd_uni, Ts);
    seg_data.InputName  = {'Current'};
    seg_data.InputUnit  = {'A'};
    seg_data.OutputName = {'Velocity'};
    seg_data.OutputUnit = {'RPS'};

    experiments{end+1} = seg_data; %#ok<SAGROW>
    fprintf('    Step %d: %.0fA, %d samples, v=%.1f-%.1f RPS\n', ...
        k, seg_cmd(1), length(seg_t_uni), min(seg_vel_uni), max(seg_vel_uni));
end

% Merge into multi-experiment dataset
id_data = merge(experiments{:});

% Estimate transfer functions
fprintf('  Estimating transfer functions...\n');
sys1 = tfest(id_data, 1);
sys2 = tfest(id_data, 2);

% Fit quality — simulate each experiment and compute overall NRMSE
all_meas = []; all_pred1 = []; all_pred2 = [];
for k = 1:length(experiments)
    exp_k = experiments{k};
    t_k = (0 : Ts : (size(exp_k.y,1)-1)*Ts)';
    all_meas  = [all_meas;  exp_k.y]; %#ok<AGROW>
    all_pred1 = [all_pred1; lsim(sys1, exp_k.u, t_k)]; %#ok<AGROW>
    all_pred2 = [all_pred2; lsim(sys2, exp_k.u, t_k)]; %#ok<AGROW>
end
nrmse1 = 100 * (1 - norm(all_meas - all_pred1) / norm(all_meas - mean(all_meas)));
nrmse2 = 100 * (1 - norm(all_meas - all_pred2) / norm(all_meas - mean(all_meas)));

fprintf('  1st-order fit: %.1f%% NRMSE\n', nrmse1);
fprintf('  2nd-order fit: %.1f%% NRMSE\n', nrmse2);

% Pick model (prefer simpler unless 2nd order is much better)
if nrmse2 > nrmse1 + 5
    sys_plant = sys2;
    model_order = 2;
    fprintf('  -> Using 2nd-order model (significantly better)\n');
else
    sys_plant = sys1;
    model_order = 1;
    fprintf('  -> Using 1st-order model\n');
end

% Extract parameters from first-order model
K_dc  = dcgain(sys1);              % RPS per Amp (steady-state gain)
poles = pole(sys1);
tau   = -1 / real(poles(1));       % Time constant (seconds)

% Derive feedforward from model
kV_model = 1 / K_dc;              % Amps / RPS
kA_model = tau / K_dc;            % Amps / (RPS/s)

fprintf('\n  1st-order model parameters:\n');
fprintf('    DC gain K = %.4f RPS/A\n', K_dc);
fprintf('    Time constant tau = %.4f s\n', tau);
fprintf('    -> kV = 1/K = %.4f A/RPS\n', kV_model);
fprintf('    -> kA = tau/K = %.4f A/(RPS/s)\n\n', kA_model);

% Display transfer functions
fprintf('  1st-order TF:\n');
disp(sys1);
if model_order == 2
    fprintf('  2nd-order TF:\n');
    disp(sys2);
end

% Plot — overlay all experiments with model predictions
figure('Name', 'System Identification', 'Position', [50 50 1400 700]);

subplot(2,2,[1 2]);
colors = lines(length(experiments));
hold on;
legend_entries = {};
for k = 1:length(experiments)
    exp_k = experiments{k};
    t_k = (0 : Ts : (size(exp_k.y,1)-1)*Ts)';
    y_k = lsim(sys_plant, exp_k.u, t_k);

    plot(t_k, exp_k.y, '-', 'Color', colors(k,:), 'LineWidth', 0.8);
    plot(t_k, y_k, '--', 'Color', colors(k,:), 'LineWidth', 1.5);
    legend_entries{end+1} = sprintf('%.0fA measured', exp_k.u(1)); %#ok<SAGROW>
    legend_entries{end+1} = sprintf('%.0fA model', exp_k.u(1)); %#ok<SAGROW>
end
ylabel('Velocity (RPS)');
xlabel('Time within step (s)');
legend(legend_entries, 'Location', 'best', 'NumColumns', 2);
title(sprintf('Model vs Measured per Step (overall fit: %.1f%%)', ...
    ternary(model_order==1, nrmse1, nrmse2)));
grid on;

subplot(2,2,3);
step(sys_plant);
title(sprintf('%s Plant Step Response (1A step)', ...
    ternary(model_order==1, '1st-order', '2nd-order')));
ylabel('Velocity (RPS)');
grid on;

subplot(2,2,4);
bode(sys_plant);
title('Plant Frequency Response');

%% ===================== COAST-DOWN VALIDATION =====================
% Free deceleration validates friction model independently.
% During coast: 0 = -kS - kV*vel + kA*accel  (no applied current)
% So: kA * |decel| = kS + kV * vel

fprintf('=== COAST-DOWN VALIDATION ===\n');

cd_mask = idx_cd & cmd_type == 2;  % Only coasting portion
cd_t   = t(cd_mask);
cd_vel = vel_rps(cd_mask);

if sum(cd_mask) > 20
    cd_t_rel = cd_t - cd_t(1);
    cd_vel_smooth = movmean(cd_vel, QS_SMOOTH_WINDOW);
    cd_decel = -gradient(cd_vel_smooth, Ts);  % Positive = slowing down

    % Filter valid range
    cd_valid = cd_vel_smooth > 1.0 & cd_decel > 0;

    if sum(cd_valid) > 10
        cd_v = cd_vel_smooth(cd_valid);
        cd_d = cd_decel(cd_valid);

        % Fit: decel = (kS/kA) + (kV/kA)*vel
        p_coast = polyfit(cd_v, cd_d, 1);
        kV_over_kA_coast = p_coast(1);
        kS_over_kA_coast = p_coast(2);

        % Using kA from model to recover absolute values
        kS_coast = kS_over_kA_coast * kA_model;
        kV_coast = kV_over_kA_coast * kA_model;

        fprintf('  Coast kS = %.4f A  (vs quasistatic %.4f)\n', kS_coast, kS_qs);
        fprintf('  Coast kV = %.4f A/RPS  (vs quasistatic %.4f)\n', kV_coast, kV_qs);
        fprintf('  Initial velocity: %.1f RPS (%.0f RPM)\n', cd_vel(1), cd_vel(1)*60);
        fprintf('  Coast duration: %.1f s\n\n', cd_t_rel(end));
    else
        fprintf('  Not enough valid coast-down points for analysis.\n\n');
    end

    figure('Name', 'Coast-Down', 'Position', [50 50 900 500]);

    subplot(2,1,1);
    plot(cd_t_rel, cd_vel, 'b', 'LineWidth', 0.8);
    xlabel('Time (s)');
    ylabel('Velocity (RPS)');
    title('Coast-Down Profile');
    grid on;

    subplot(2,1,2);
    if sum(cd_valid) > 10
        scatter(cd_v, cd_d, 10, 'b', 'filled', 'MarkerFaceAlpha', 0.3);
        hold on;
        v_fit = linspace(min(cd_v), max(cd_v), 100);
        plot(v_fit, polyval(p_coast, v_fit), 'r-', 'LineWidth', 2);
        legend('Data', 'Linear fit');
    end
    xlabel('Velocity (RPS)');
    ylabel('Deceleration (RPS/s)');
    title('Deceleration vs Velocity');
    grid on;
else
    fprintf('  Insufficient coast-down data.\n\n');
end

%% ===================== VOLTAGE STEPS ANALYSIS =====================
% Secondary analysis: voltage step data for additional validation

fprintf('=== VOLTAGE STEPS (Supplementary) ===\n');

vs_t   = t(idx_vs);
vs_vel = vel_rps(idx_vs);
vs_cmd = cmd_value(idx_vs);
vs_ct  = cmd_type(idx_vs);
vs_i   = stator_i(idx_vs);
vs_mv  = motor_v(idx_vs);

% Identify voltage-to-velocity plant
vs_input = vs_cmd;
vs_input(vs_ct == 2) = 0;

if length(vs_vel) > 20
    vs_t_rel = vs_t - vs_t(1);
    vs_t_uniform = (0 : Ts : vs_t_rel(end))';
    vs_vel_uniform   = interp1(vs_t_rel, vs_vel,   vs_t_uniform, 'linear', 'extrap');
    vs_input_uniform = interp1(vs_t_rel, vs_input, vs_t_uniform, 'previous', 'extrap');

    id_data_v = iddata(vs_vel_uniform, vs_input_uniform, Ts);
    id_data_v.InputName  = {'Voltage'};
    id_data_v.InputUnit  = {'V'};
    id_data_v.OutputName = {'Velocity'};
    id_data_v.OutputUnit = {'RPS'};

    sys_v1 = tfest(id_data_v, 1);
    K_v = dcgain(sys_v1);
    fprintf('  Voltage-domain DC gain: %.4f RPS/V\n', K_v);
    fprintf('  (For reference only - current-domain gains used for CTRE)\n\n');
end

%% ===================== PID AUTO-TUNING =====================
% pidtune on current->velocity plant gives gains in CTRE units directly:
%   kP = Amps / RPS  (output current per RPS of velocity error)
%   kI = Amps / rotation  (output current per rotation of integrated error)
%   kD = Amps / (RPS/s)  (output current per RPS/s of error derivative)

fprintf('=== PID AUTO-TUNING ===\n');
fprintf('  Target bandwidth: %.0f rad/s\n', PID_BANDWIDTH);
fprintf('  Target phase margin: %.0f deg\n\n', PID_PHASE_MARGIN);

opts = pidtuneOptions('PhaseMargin', PID_PHASE_MARGIN);

% Full PID
[C_pid, info_pid] = pidtune(sys_plant, 'PID', PID_BANDWIDTH, opts);
kP_pid = C_pid.Kp;
kI_pid = C_pid.Ki;
kD_pid = C_pid.Kd;

fprintf('  PID result:\n');
fprintf('    kP = %.4f A/RPS\n', kP_pid);
fprintf('    kI = %.4f A/rotation\n', kI_pid);
fprintf('    kD = %.4f A/(RPS/s)\n', kD_pid);
fprintf('    Achieved PM: %.1f deg\n\n', info_pid.PhaseMargin);

% PI alternative (no derivative)
[C_pi, info_pi] = pidtune(sys_plant, 'PI', PID_BANDWIDTH, opts);
fprintf('  PI alternative (if D causes noise issues):\n');
fprintf('    kP = %.4f, kI = %.4f\n', C_pi.Kp, C_pi.Ki);
fprintf('    Achieved PM: %.1f deg\n\n', info_pi.PhaseMargin);

% Closed-loop analysis
cl_pid = feedback(C_pid * sys_plant, 1);
cl_pi  = feedback(C_pi * sys_plant, 1);

% Step response metrics
s_info = stepinfo(cl_pid);
fprintf('  Closed-loop step response (PID):\n');
fprintf('    Rise time:     %.3f s\n', s_info.RiseTime);
fprintf('    Settling time: %.3f s\n', s_info.SettlingTime);
fprintf('    Overshoot:     %.1f%%\n', s_info.Overshoot);
fprintf('    Steady-state:  %.4f\n\n', dcgain(cl_pid));

% Plot PID results
figure('Name', 'PID Tuning Results', 'Position', [50 50 1400 700]);

subplot(2,2,1);
step(cl_pid);
hold on;
step(cl_pi);
legend('PID', 'PI');
title('Closed-Loop Step Response (1 RPS setpoint change)');
ylabel('Velocity (RPS)');
grid on;

subplot(2,2,2);
margin(C_pid * sys_plant);
title('Open-Loop Bode: PID * Plant');

subplot(2,2,3);
% Disturbance rejection at plant input (e.g. friction change)
S_pid = feedback(1, C_pid * sys_plant);
step(S_pid);
title('Disturbance Rejection (Input Sensitivity)');
ylabel('Velocity Error (RPS)');
grid on;

subplot(2,2,4);
% Simulate a velocity tracking scenario
t_sim = 0:Ts:5;
ref = zeros(size(t_sim));
ref(t_sim >= 0.5) = 50;  % Step to 50 RPS (3000 RPM)
ref(t_sim >= 3.0) = 70;  % Step to 70 RPS (4200 RPM)
y_sim = lsim(cl_pid, ref, t_sim);
plot(t_sim, ref, 'k--', 'LineWidth', 1);
hold on;
plot(t_sim, y_sim, 'b', 'LineWidth', 1.2);
xlabel('Time (s)');
ylabel('Velocity (RPS)');
legend('Setpoint', 'Response');
title('Velocity Tracking Simulation');
grid on;

%% ===================== FINAL RESULTS =====================
% Primary feedforward: quasistatic regression (best data coverage)
% Primary kA: from identified model time constant (more robust than regression kA)
% PID: from pidtune

kS_final = kS_qs;
kV_final = kV_qs;
kA_final = kA_model;
kP_final = kP_pid;
kI_final = kI_pid;
kD_final = kD_pid;

fprintf('\n');
fprintf('================================================================\n');
fprintf('   FINAL GAINS - CTRE VelocityTorqueCurrentFOC                 \n');
fprintf('   Ready to paste into Constants.java FlywheelConstants         \n');
fprintf('================================================================\n');
fprintf('   Feedforward                                                  \n');
fprintf('     S (kS) = %8.4f   Amps              (was %.4f)\n', kS_final, EXISTING_KS);
fprintf('     V (kV) = %8.4f   Amps / RPS        (was %.4f)\n', kV_final, EXISTING_KV);
fprintf('     A (kA) = %8.4f   Amps / (RPS/s)    (was %.4f)\n', kA_final, EXISTING_KA);
fprintf('   PID                                                          \n');
fprintf('     P (kP) = %8.4f   Amps / RPS        (was %.4f)\n', kP_final, EXISTING_KP);
fprintf('     I (kI) = %8.4f   Amps / rotation   (was %.4f)\n', kI_final, EXISTING_KI);
fprintf('     D (kD) = %8.4f   Amps / (RPS/s)    (was %.4f)\n', kD_final, EXISTING_KD);
fprintf('================================================================\n');

fprintf('\n// --- Copy into Constants.java FlywheelConstants ---\n');
fprintf('public static final double S = %.4f;\n', kS_final);
fprintf('public static final double V = %.4f;\n', kV_final);
fprintf('public static final double A = %.4f;\n', kA_final);
fprintf('public static final double P = %.4f;\n', kP_final);
fprintf('public static final double I = %.4f;\n', kI_final);
fprintf('public static final double D = %.4f;\n', kD_final);

%% ===================== CROSS-VALIDATION =====================
fprintf('\n=== CROSS-VALIDATION ===\n');
fprintf('  kS: quasistatic=%.4f  coast=', kS_qs);
if exist('kS_coast', 'var'), fprintf('%.4f', kS_coast); else, fprintf('N/A'); end
fprintf('\n');
fprintf('  kV: quasistatic=%.4f  model=%.4f  coast=', kV_qs, kV_model);
if exist('kV_coast', 'var'), fprintf('%.4f', kV_coast); else, fprintf('N/A'); end
fprintf('\n');
fprintf('  kA: model=%.4f\n', kA_model);
fprintf('  kV agreement (qs vs model): %.1f%% difference\n', ...
    abs(kV_qs - kV_model) / max(abs(kV_qs), 1e-6) * 100);
if abs(kV_qs - kV_model) / max(abs(kV_qs), 1e-6) > 0.3
    fprintf('  NOTE: >30%% kV disagreement. Check quasistatic plot for data quality.\n');
    fprintf('  If model kV is closer to your existing value, consider using it instead.\n');
end

%% ===================== HELPER FUNCTIONS =====================
function s = ternary(cond, a, b)
    if cond, s = a; else, s = b; end
end
