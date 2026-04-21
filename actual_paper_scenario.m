%% CAV Platoon Collision Avoidance with Split-Merge - Based on Research Paper
% 2 SVs (Leader SV1, Follower SV2) in middle lane
% 1 OV in left lane cutting to right lane between them
% Paper: "Collision Avoidance Motion Planning for CAV Platoon Merging and Splitting"

clear; close all; clc;

%% Simulation Parameters (Table III from paper)
dt = 0.05;              % Time step (s)
T_sim = 25;             % Total simulation time (s)
t = 0:dt:T_sim;
N = length(t);

% Vehicle parameters
m = 1500;               % Mass (kg)
lf = 1.434;             % Front axle to CG (m)
lr = 1.421;             % Rear axle to CG (m)
Iz = 4600;              % Moment of inertia (kg·m²)
Cf = 127000;            % Front cornering stiffness (N/rad)
Cr = 127000;            % Rear cornering stiffness (N/rad)
v_max = 30;             % Max speed (m/s)
a_max = 3;              % Max acceleration (m/s²)
a_min = -5;             % Max deceleration (m/s²)

% CRPF parameters (Equation 11)
G = 0.5;
R = 1;
K = 5;
T_veh = 1;
zeta = 1.2;

% Collision warning threshold (from paper Section VI.A)
TTC_threshold = 2.2;    % Time-to-collision threshold (s)
CRPF_threshold = 300;   % Collision risk potential field threshold

% Platoon parameters
desired_headway = 6;    % Desired spacing (m)
platoon_speed = 25.4;     % Desired platoon speed (m/s)

% Lane parameters
lane_width = 4;         % Lane width (m)
lane_left = -lane_width;
lane_mid = 0;
lane_right = lane_width;

%% Initialize Vehicles
% SV1 (Leader) - Middle lane
SV1.x = 0;              % Longitudinal position
SV1.y = lane_mid;       % Lateral position
SV1.vx = platoon_speed; % Longitudinal velocity
SV1.vy = 0;             % Lateral velocity
SV1.theta = 0;          % Heading angle
SV1.ax = 0;             % Acceleration
SV1.in_platoon = true;

% SV2 (Follower) - Middle lane
SV2.x = -desired_headway;
SV2.y = lane_mid;
SV2.vx = platoon_speed;
SV2.vy = 0;
SV2.theta = 0;
SV2.ax = 0;
SV2.in_platoon = true;

% OV (Obstacle) - Left lane, ahead
OV.x = 20;              % Start ahead
OV.y = lane_left;       % Left lane
OV.vx = 22;             % Slightly slower longitudinal speed
OV.vy = 0;              % No lateral motion initially
OV.theta = 0;
OV.ay = 0;              % Lateral acceleration
OV.lane_change_start = 5;   % Start lane change at t=5s
OV.lane_change_duration = 3; % Duration of lane change

% Storage arrays
SV1_traj = zeros(N, 6);  % [x, y, vx, vy, theta, ax]
SV2_traj = zeros(N, 6);
OV_traj = zeros(N, 6);
CRPF_SV1 = zeros(N, 1);
CRPF_SV2 = zeros(N, 1);
TTC_SV1 = zeros(N, 1);
TTC_SV2 = zeros(N, 1);
platoon_status = true(N, 1);  % Platoon connected status

%% Main Simulation Loop
for k = 1:N
    current_time = t(k);
    
    %% OV Lane Change Maneuver (sinusoidal lateral motion)
    if current_time >= OV.lane_change_start && ...
       current_time < OV.lane_change_start + OV.lane_change_duration
        
        % Smooth lane change from left to right (sine-based trajectory)
        progress = (current_time - OV.lane_change_start) / OV.lane_change_duration;
        target_y = lane_left + (lane_right - lane_left) * (1 - cos(pi * progress)) / 2;
        OV.vy = (target_y - OV.y) / dt;
        OV.ay = 0.5;  % Lateral acceleration during lane change
    else
        OV.vy = 0;
        OV.ay = 0;
    end
    
    %% Calculate CRPF for both SVs (Equation 11 from paper)
    % CRPF for SV1
    dx1 = OV.x - SV1.x;
    dy1 = OV.y - SV1.y;
    rd1 = sqrt(dx1^2 + dy1^2);
    
    if rd1 > 0.1
        M_OV = m * (1.566e-14 * OV.vx^6.687 + 0.3354);
        CRPF_val1 = (K * G * T_veh * R * M_OV) / (rd1^zeta);
        CRPF_SV1(k) = CRPF_val1;
        
        % TTC calculation
        rel_vx1 = SV1.vx - OV.vx;
        rel_vy1 = SV1.vy - OV.vy;
        rel_v1 = sqrt(rel_vx1^2 + rel_vy1^2);
        if rel_v1 > 0
            TTC_SV1(k) = rd1 / rel_v1;
        else
            TTC_SV1(k) = 999;
        end
    else
        CRPF_SV1(k) = 0;
        TTC_SV1(k) = 999;
    end
    
    % CRPF for SV2
    dx2 = OV.x - SV2.x;
    dy2 = OV.y - SV2.y;
    rd2 = sqrt(dx2^2 + dy2^2);
    
    if rd2 > 0.1
        CRPF_val2 = (K * G * T_veh * R * M_OV) / (rd2^zeta);
        CRPF_SV2(k) = CRPF_val2;
        
        rel_vx2 = SV2.vx - OV.vx;
        rel_vy2 = SV2.vy - OV.vy;
        rel_v2 = sqrt(rel_vx2^2 + rel_vy2^2);
        if rel_v2 > 0
            TTC_SV2(k) = rd2 / rel_v2;
        else
            TTC_SV2(k) = 999;
        end
    else
        CRPF_SV2(k) = 0;
        TTC_SV2(k) = 999;
    end
    
    %% Hybrid Automaton: Platoon Split/Merge Decision (Section V)
    collision_risk_SV1 = (CRPF_SV1(k) > CRPF_threshold) || (TTC_SV1(k) < TTC_threshold);
    collision_risk_SV2 = (CRPF_SV2(k) > CRPF_threshold) || (TTC_SV2(k) < TTC_threshold);
    
    % Split condition: Either SV detects high collision risk
    if (collision_risk_SV1 || collision_risk_SV2) && SV1.in_platoon
        SV1.in_platoon = false;
        SV2.in_platoon = false;
        fprintf('PLATOON SPLIT at t=%.2fs (CRPF_SV1=%.1f, CRPF_SV2=%.1f)\n', ...
                current_time, CRPF_SV1(k), CRPF_SV2(k));
    end
    
    % Merge condition: Both SVs safe AND OV passed between them
    SV_spacing = SV1.x - SV2.x;
    OV_between = (OV.x < SV1.x) && (OV.x > SV2.x);
    OV_cleared = (OV.x > SV1.x + 5) || (OV.x < SV2.x - 5);  % OV fully clear
    
    if ~SV1.in_platoon && ~collision_risk_SV1 && ~collision_risk_SV2 && ...
       OV_cleared && abs(SV_spacing - desired_headway) < 3
        SV1.in_platoon = true;
        SV2.in_platoon = true;
        fprintf('PLATOON MERGE at t=%.2fs (Spacing=%.2fm)\n', ...
                current_time, SV_spacing);
    end
    
    platoon_status(k) = SV1.in_platoon;
    
    %% MPC-Based Motion Planning (Equations 30-31)
    % SV1 Control Logic
    if SV1.in_platoon
        % Platooning mode: maintain desired speed and lateral position
        a_des_SV1 = 2.0 * (platoon_speed - SV1.vx);  % Velocity tracking
        a_des_SV1 = a_des_SV1 - 0.5 * SV1.y;  % Lateral correction
    else
        % Adaptive cruise: avoid collision
        if collision_risk_SV1
            % Speed up to escape (Equation 30 - single vehicle cruising)
            a_des_SV1 = a_max * 0.8;  % Aggressive acceleration
        else
            % Return to desired speed
            a_des_SV1 = 1.5 * (platoon_speed - SV1.vx);
        end
    end
    
    % SV2 Control Logic
    if SV2.in_platoon
        % Platooning mode: follow SV1 with desired headway (Equation 31)
        spacing_error = (SV1.x - SV2.x) - desired_headway;
        vel_error = SV1.vx - SV2.vx;
        a_des_SV2 = 1.2 * spacing_error + 0.8 * vel_error;
        a_des_SV2 = a_des_SV2 - 0.5 * SV2.y;  % Lateral correction
    else
        % Adaptive cruise: avoid collision
        if collision_risk_SV2
            % Slow down to create gap (Equation 30)
            a_des_SV2 = a_min * 0.6;  % Moderate braking
        else
            % Catch up to SV1
            spacing_error = (SV1.x - SV2.x) - desired_headway;
            a_des_SV2 = 1.0 * spacing_error;
        end
    end
    
    % Apply constraints (Equations 41-44)
    SV1.ax = max(a_min, min(a_max, a_des_SV1));
    SV2.ax = max(a_min, min(a_max, a_des_SV2));
    
    %% Update Vehicle States (Equations 20-21, 24-27)
    % SV1 update
    SV1.vx = SV1.vx + SV1.ax * dt;
    SV1.vx = max(0, min(v_max, SV1.vx));
    SV1.x = SV1.x + SV1.vx * dt;
    SV1.y = SV1.y + SV1.vy * dt;
    
    % Lateral stabilization (keep in middle lane)
    SV1.vy = -0.5 * SV1.y;
    
    % SV2 update
    SV2.vx = SV2.vx + SV2.ax * dt;
    SV2.vx = max(0, min(v_max, SV2.vx));
    SV2.x = SV2.x + SV2.vx * dt;
    SV2.y = SV2.y + SV2.vy * dt;
    
    % Lateral stabilization
    SV2.vy = -0.5 * SV2.y;
    
    % OV update
    OV.vx = OV.vx + 0 * dt;  % Constant longitudinal speed
    OV.x = OV.x + OV.vx * dt;
    OV.y = OV.y + OV.vy * dt;
    
    % Store trajectories
    SV1_traj(k, :) = [SV1.x, SV1.y, SV1.vx, SV1.vy, SV1.theta, SV1.ax];
    SV2_traj(k, :) = [SV2.x, SV2.y, SV2.vx, SV2.vy, SV2.theta, SV2.ax];
    OV_traj(k, :) = [OV.x, OV.y, OV.vx, OV.vy, OV.theta, 0];
end

%% Visualization (Similar to Figures 12-15 in paper)
figure('Position', [50 50 1400 900]);

% Subplot 1: Trajectory (Fig 12b, 14b)
subplot(3,3,1);
hold on; grid on;
plot(SV1_traj(:,1), SV1_traj(:,2), 'b-', 'LineWidth', 2);
plot(SV2_traj(:,1), SV2_traj(:,2), 'g-', 'LineWidth', 2);
plot(OV_traj(:,1), OV_traj(:,2), 'r--', 'LineWidth', 2);

% Plot lane boundaries
yline(lane_left - lane_width/2, 'k--', 'LineWidth', 1);
yline(lane_left + lane_width/2, 'k--', 'LineWidth', 1);
yline(lane_mid - lane_width/2, 'k--', 'LineWidth', 1);
yline(lane_mid + lane_width/2, 'k--', 'LineWidth', 1);
yline(lane_right - lane_width/2, 'k--', 'LineWidth', 1);
yline(lane_right + lane_width/2, 'k--', 'LineWidth', 1);

xlabel('X (m)'); ylabel('Y (m)');
title('Motion Trajectory (Fig 12b)');
legend('SV1 (Leader)', 'SV2 (Follower)', 'OV (Obstacle)');
axis equal;

% Subplot 2: CRPF Strength (Fig 12a, 14a)
subplot(3,3,2);
hold on; grid on;
plot(t, CRPF_SV1, 'b-', 'LineWidth', 1.5);
plot(t, CRPF_SV2, 'g-', 'LineWidth', 1.5);
yline(CRPF_threshold, 'r--', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('CRPF (Ev)');
title('Collision Risk Potential Field (Fig 12a)');
legend('SV1', 'SV2', 'Threshold');


% Subplot 3: Headway (Fig 12c, 14c)
subplot(3,3,3);
headway = SV1_traj(:,1) - SV2_traj(:,1);
hold on; grid on;
plot(t, headway, 'b-', 'LineWidth', 1.5);
yline(desired_headway, 'r--', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Headway (m)');
title('Space Headway (Fig 12c)');
legend('Actual', 'Desired');

% Subplot 4: Lateral Position
subplot(3,3,4);
hold on; grid on;
plot(t, SV1_traj(:,2), 'b-', 'LineWidth', 1.5);
plot(t, SV2_traj(:,2), 'g-', 'LineWidth', 1.5);
plot(t, OV_traj(:,2), 'r--', 'LineWidth', 1.5);
yline(lane_mid, 'k--');
xlabel('Time (s)'); ylabel('Y (m)');
title('Lateral Position (Fig 12d)');
legend('SV1', 'SV2', 'OV', 'Lane Center');

% Subplot 5: Platoon Status (Hybrid Automaton State - Fig 7)
subplot(3,3,5);
hold on; grid on;
plot(t, platoon_status, 'b-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Status');
title('Platoon Status (Hybrid Automaton)');
yticks([0 1]); yticklabels({'Split', 'Merged'});
ylim([-0.1 1.1]);

% Subplot 6: Relative Positions
subplot(3,3,6);
hold on; grid on;
rel_x_SV1 = OV_traj(:,1) - SV1_traj(:,1);
rel_x_SV2 = OV_traj(:,1) - SV2_traj(:,1);
plot(t, rel_x_SV1, 'b-', 'LineWidth', 1.5);
plot(t, rel_x_SV2, 'g-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Relative X (m)');
title('Relative Longitudinal Position');
legend('OV-SV1', 'OV-SV2');
yline(0, 'r--');

sgtitle('CAV Platoon Collision Avoidance: Split-Merge Maneuver', 'FontSize', 14, 'FontWeight', 'bold');

%% Animation
figure('Position', [100 100 1200 600]);
for k = 1:5:N
    clf;
    hold on; grid on;
    
    % Draw lanes
    rectangle('Position', [-50, lane_left-lane_width/2, 700, lane_width], ...
              'FaceColor', [0.9 0.9 0.9], 'EdgeColor', 'k');
    rectangle('Position', [-50, lane_mid-lane_width/2, 700, lane_width], ...
              'FaceColor', [0.95 0.95 0.95], 'EdgeColor', 'k');
    rectangle('Position', [-50, lane_right-lane_width/2, 700, lane_width], ...
              'FaceColor', [0.9 0.9 0.9], 'EdgeColor', 'k');
    
    % Draw vehicles
    draw_vehicle(SV1_traj(k,1), SV1_traj(k,2), 'b', 'SV1');
    draw_vehicle(SV2_traj(k,1), SV2_traj(k,2), 'g', 'SV2');
    draw_vehicle(OV_traj(k,1), OV_traj(k,2), 'r', 'OV');
    
    % Draw CRPF zones
    if CRPF_SV1(k) > CRPF_threshold
        circle(OV_traj(k,1), OV_traj(k,2), 10, 'r', 0.2);
    end
    
    % Status text
    if platoon_status(k)
        status_text = 'PLATOON MODE';
        color = 'green';
    else
        status_text = 'SPLIT MODE';
        color = 'red';
    end
    
    text(SV1_traj(k,1), 10, status_text, 'FontSize', 14, ...
         'FontWeight', 'bold', 'Color', color);
    
    text(SV1_traj(k,1), -10, sprintf('t=%.2fs', t(k)), 'FontSize', 12);
    text(SV1_traj(k,1), -12, sprintf('CRPF: SV1=%.1f, SV2=%.1f', ...
         CRPF_SV1(k), CRPF_SV2(k)), 'FontSize', 10);
    
    xlim([SV1_traj(k,1)-60, SV1_traj(k,1)+60]);
    ylim([-8, 8]);
    xlabel('X (m)'); ylabel('Y (m)');
    title('Real-time CAV Platoon Collision Avoidance Animation');
    
    drawnow;
    pause(0.01);
end

%% Helper Functions
function draw_vehicle(x, y, color, label)
    % Draw vehicle as rectangle
    L = 2.5; W = 1;
    rectangle('Position', [x-L/2, y-W/2, L, W], ...
              'FaceColor', color, 'EdgeColor', 'k', 'LineWidth', 2);
    text(x, y, label, 'HorizontalAlignment', 'center', ...
         'Color', 'w', 'FontWeight', 'bold');
end

function circle(x, y, r, color, alpha)
    % Draw circle for CRPF visualization
    theta = linspace(0, 2*pi, 50);
    fill(x + r*cos(theta), y + r*sin(theta), color, ...
         'FaceAlpha', alpha, 'EdgeColor', 'none');
end

fprintf('\n=== SIMULATION COMPLETE ===\n');
fprintf('Check the figures for trajectory, CRPF, and animation results.\n');