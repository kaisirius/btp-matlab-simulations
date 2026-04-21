function cross_scenario()
% Scenario: 2 SVs (LEFT lane) want to reach RIGHT lane
% OV (MIDDLE lane, faster) blocks SV2's crossing
% SV1 (LEADER) crosses successfully LEFT→MIDDLE→RIGHT from FRONT of OV
% SV2 (FOLLOWER) WAITS in LEFT lane, lets OV pass, then crosses from BEHIND

rng(42); 
OUT = "outputs_cross";
if ~exist(OUT,"dir"), mkdir(OUT); end

%% --- Global Configuration
dt = 0.10; 
T = 90.0;  % Extended time to ensure completion
N = round(T/dt);

% Road: 3 lanes (LEFT, MIDDLE, RIGHT)
lane_center_left   = -4.0;
lane_center_middle =  0.0;
lane_center_right  = +4.0;
lane_width = 4.0;

% Safety radii
rS = 1.0; 
rO = 1.0; 
Rsum = rS + rO;

% Tracks (reference lines)
TRACK_LEFT   = [lane_center_left,   -500; lane_center_left,   +500];
TRACK_MIDDLE = [lane_center_middle, -500; lane_center_middle, +500];
TRACK_RIGHT  = [lane_center_right,  -500; lane_center_right,  +500];

% Platoon parameters
v_des_platoon   = 12.0;   % m/s (SVs slower than OV)
d_headway_des   = 20.0;   % m
merge_headway_tol = 4.0;  % m

% Longitudinal limits
a_min = -3.5; 
a_max = +5.0;
v_min = 3.0;  
v_max = 30.0;

% Lane change timing
TAU_LC = 2.5;          % duration of lane change (seconds)
HOLD_TICKS = 3;        % Reduced stabilization ticks

% OV parameters (faster speed, MIDDLE lane, starts behind)
ov_x  = lane_center_middle;
ov_y  = -60.0;         % starts 40m behind SV1
ov_vx = 0.0; 
ov_vy = 16.0;          % 16 m/s (faster than SVs at 12 m/s)
m_O = 1800.0; 
size_O = 5.0; 
kappa_O = 1.2;

% CRPF parameters (Section III-B, Equation 11)
CRPF_G    = 0.5;
CRPF_ZETA = 1.2;
PSEUDO_EPS = 1.0;
PSEUDO_RHO = 0.02;

% Risk thresholds (Section VI-A)
CR_YELLOW_LOW  = 0.006;
CR_YELLOW_HIGH = 0.010;
TTC_SPLIT = 2.5;       % Split trigger threshold
TTC_MERGE = 4.0;       % Merge condition threshold

% Clearance for lane change decisions
Y_CLEAR_AHEAD  = 15.0;  % OV must be this far ahead
Y_SAFE_BEHIND  = 12.0;  % OV must be this far behind for front crossing
Y_OV_PASSED    = 30.0;  % OV must be this far ahead for rear crossing

% DWA parameters (Section III-C)
dwa_steps = 10;        % prediction horizon
W_RISK  = 1.5;
W_SPEED = 0.12;
W_HEADWAY = 0.8;       % for platoon mode

%% --- Initialize 2 SVs on LEFT lane
nSV = 2;
SV = repmat(struct(...
    'x', 0, 'y', 0, 'v', v_des_platoon, 'theta', pi/2, ...
    'mode', "PLATOON", 'lane_target', "RIGHT", 'track', TRACK_LEFT, ...
    'preced', 0, 'color', [0 0 0], ...
    'split_time', NaN, 'merge_time', NaN, ...
    'recenter', false, 'lock_lane', false, 'ok_ctr', 0, ...
    'lane_cur', "LEFT", 'lc_active', false, 'lc_t', 0, 'lc_T', TAU_LC, ...
    'lane_from', "LEFT", 'lane_to', "LEFT", ...
    'crossed_to_right', false, 'ov_clear', false), nSV, 1);

% Initial positions: both on LEFT lane
lead_y = 0.0; 
gap0 = d_headway_des;
for i = 1:nSV
    SV(i).x = lane_center_left;
    SV(i).y = lead_y - (i-1)*gap0;  % SV1 at y=0, SV2 at y=-20
    SV(i).v = v_des_platoon;
    SV(i).theta = pi/2;
    SV(i).preced = (i > 1) * (i-1);  % SV2's predecessor is SV1
    SV(i).lane_cur = "LEFT";
    SV(i).lane_target = "RIGHT";     % goal: reach RIGHT lane
end

% Colors for visualization
cols = lines(nSV); 
for i = 1:nSV
    SV(i).color = cols(i,:); 
end

% Logging structures
log(nSV) = struct('t',[],'x',[],'y',[],'v',[],'theta',[],...
    'riskOV',[],'gap',[],'relv',[],'mode',strings(0,1),'lane',strings(0,1));
for i = 1:nSV
    log(i) = struct('t',[],'x',[],'y',[],'v',[],'theta',[],...
        'riskOV',[],'gap',[],'relv',[],'mode',strings(0,1),'lane',strings(0,1));
end
logOV = struct('t',[],'x',[],'y',[],'vy',[]);

% Track OV position when SVs REACH MIDDLE lane (completion events)
middle_reach_events = struct('sv_id', {}, 't', {}, 'sv_y', {}, 'ov_x', {}, 'ov_y', {});

collision = false;

%% --- Main Simulation Loop
fprintf('\n========== STARTING SIMULATION ==========\n');
fprintf('Scenario: 2 SVs (LEFT) → RIGHT, OV (MIDDLE, faster) blocks SV2\n\n');

for k = 1:N
    t = (k-1)*dt;
    
    % OV kinematics (constant velocity on MIDDLE lane)
    ov_x = ov_x + ov_vx*dt;
    ov_y = ov_y + ov_vy*dt;
    
    %% --- CHECK IF OV HAS CLEARED EACH SV
    for i = 1:nSV
        % OV is "clear" if it's far enough ahead
        if (ov_y - SV(i).y) > Y_OV_PASSED && ~SV(i).ov_clear
            SV(i).ov_clear = true;
            fprintf('  ✓ t=%.1fs: OV cleared SV%d (OV ahead by %.1fm)\n', ...
                t, i, ov_y - SV(i).y);
        end
    end
    
    %% --- LANE DECISION LOGIC (Hybrid Automaton Section V)
    for i = 1:nSV
        % Skip if currently executing lane change
        if SV(i).lc_active
            SV(i).lane_target = SV(i).lane_to;
            SV(i).track = lane_track_of(SV(i).lane_to, TRACK_LEFT, TRACK_MIDDLE, TRACK_RIGHT);
            continue
        end
        
        % Skip if locked (stabilization period)
        if SV(i).lock_lane
            continue
        end
        
        % Decision tree based on current lane
        cur_lane = SV(i).lane_cur;
        
        if cur_lane == "LEFT"
            % *** CRITICAL FIX: LEADER vs FOLLOWER behavior ***
            
            if SV(i).preced == 0
                % LEADER (SV1): Can cross from FRONT if maintaining lead over OV
                if (SV(i).y - ov_y) > Y_SAFE_BEHIND
                    next_lane = "MIDDLE";
                    if can_change_lane_smart(i, next_lane, SV, ov_x, ov_y, ov_vy, ...
                            lane_center_middle, Y_CLEAR_AHEAD, Y_SAFE_BEHIND, Rsum, ...
                            d_headway_des)
                        start_lane_change(i, next_lane, "FRONT");
                    end
                end
            else
                % FOLLOWER (SV2+): MUST WAIT until OV has completely passed (ov_clear flag)
                if SV(i).ov_clear
                    next_lane = "MIDDLE";
                    if can_change_lane_smart(i, next_lane, SV, ov_x, ov_y, ov_vy, ...
                            lane_center_middle, Y_CLEAR_AHEAD, Y_SAFE_BEHIND, Rsum, ...
                            d_headway_des)
                        start_lane_change(i, next_lane, "BEHIND");
                    end
                else
                    % SV2 waiting in LEFT lane for OV to pass
                    if mod(k, 50) == 0
                        fprintf('  ⏳ t=%.1fs: SV%d waiting in LEFT (OV distance: %.1fm)\n', ...
                            t, i, ov_y - SV(i).y);
                    end
                end
            end
            
        elseif cur_lane == "MIDDLE"
            % Goal: MIDDLE → RIGHT
            next_lane = "RIGHT";
            if can_change_lane_smart(i, next_lane, SV, ov_x, ov_y, ov_vy, ...
                    lane_center_right, Y_CLEAR_AHEAD, Y_SAFE_BEHIND, Rsum, ...
                    d_headway_des)
                start_lane_change(i, next_lane, "");
                SV(i).crossed_to_right = true;
            end
            
        elseif cur_lane == "RIGHT"
            % Already at target - stay here
            SV(i).lane_target = "RIGHT";
            SV(i).track = TRACK_RIGHT;
        end
    end
    
    %% --- LONGITUDINAL CONTROL (DWA + CRPF, Section IV)
    for i = 1:nSV
        vxi = 0;
        vyi = SV(i).v;
        
        % Collision risk & TTC to OV (Section III-A & III-B)
        p_rel_OV = [SV(i).x - ov_x, SV(i).y - ov_y];
        v_rel_OV = [vxi - ov_vx, vyi - ov_vy];
        risk_OV = crpf(SV(i).x, SV(i).y, ov_x, ov_y, hypot(ov_vx, ov_vy), 0, 0, ...
            m_O, size_O, kappa_O, CRPF_G, CRPF_ZETA, PSEUDO_EPS, PSEUDO_RHO);
        [~, ttcOV] = will_collide(p_rel_OV, v_rel_OV, Rsum, 10.0);
        
        % TTC to predecessor
        if SV(i).preced > 0
            j = SV(i).preced;
            p_rel_P = [0, SV(i).y - SV(j).y];
            v_rel_P = [0, SV(i).v - SV(j).v];
            [~, ttcP] = will_collide(p_rel_P, v_rel_P, 2*Rsum, 10.0);
            gap_to_pred = SV(j).y - SV(i).y;
        else
            ttcP = inf;
            gap_to_pred = inf;
        end
        
        %% MODE TRANSITIONS (Hybrid Automaton, Section V-C)
        
        % SPLIT condition (Table I)
        need_split = (ttcOV < TTC_SPLIT) || (ttcP < 1.8) || (risk_OV >= CR_YELLOW_HIGH);
        if SV(i).mode == "PLATOON" && need_split
            SV(i).mode = "CRUISE";
            SV(i).split_time = t;
            fprintf('  ⚠️  t=%.1fs: SV%d SPLITS (TTC_OV=%.2f, Risk=%.4f)\n', ...
                t, i, ttcOV, risk_OV);
        end
        
        % MERGE condition (Table II) - only if both on RIGHT and safe
        if SV(i).mode == "CRUISE" && SV(i).preced > 0
            j = SV(i).preced;
            both_on_right = (SV(i).lane_cur == "RIGHT") && (SV(j).lane_cur == "RIGHT");
            safe_gap = (gap_to_pred > d_headway_des - merge_headway_tol) && ...
                       (gap_to_pred < d_headway_des + 2*merge_headway_tol);
            low_risk = (risk_OV <= CR_YELLOW_LOW);
            good_ttc = (ttcOV > TTC_MERGE);
            
            if both_on_right && safe_gap && low_risk && good_ttc && ~SV(i).lc_active
                SV(i).mode = "PLATOON";
                SV(i).merge_time = t;
                fprintf('  ✅ t=%.1fs: SV%d MERGES back to platoon\n', t, i);
            end
        end
        
        %% DWA: Dynamic Window Approach (Section III-C)
        a_samples = linspace(a_min, a_max, 9);
        alpha = gate_alpha(risk_OV, CR_YELLOW_LOW, CR_YELLOW_HIGH);
        
        best_cost = inf;
        best_v_next = SV(i).v;
        
        for a_cmd = a_samples
            v_next = min(max(SV(i).v + a_cmd*dt, v_min), v_max);
            
            % Rollout prediction (longitudinal only, Equation 13)
            y_tmp = SV(i).y;
            acc_risk = 0;
            for s = 1:dwa_steps
                y_tmp = y_tmp + v_next*dt;
                ov_y_s = ov_y + ov_vy*(s*dt);
                ov_x_s = ov_x; % OV stays in MIDDLE
                rr = crpf(SV(i).x, y_tmp, ov_x_s, ov_y_s, hypot(ov_vx, ov_vy), 0, 0, ...
                    m_O, size_O, kappa_O, CRPF_G, CRPF_ZETA, PSEUDO_EPS, PSEUDO_RHO);
                acc_risk = acc_risk + rr;
            end
            mean_risk = acc_risk / dwa_steps;
            
            % Cost function (Equations 30-31)
            cost_risk = W_RISK * alpha * mean_risk;
            cost_speed = W_SPEED * (v_des_platoon - v_next)^2;
            
            % Platoon mode adds headway term
            if SV(i).mode == "PLATOON" && SV(i).preced > 0
                pred_gap = SV(SV(i).preced).y - SV(i).y;
                cost_headway = W_HEADWAY * (pred_gap - d_headway_des)^2;
            else
                cost_headway = 0;
            end
            
            total_cost = cost_risk + cost_speed + cost_headway;
            
            if total_cost < best_cost
                best_cost = total_cost;
                best_v_next = v_next;
            end
        end
        
        % Apply chosen velocity
        SV(i).v = best_v_next;
        SV(i).theta = pi/2; % straight heading
    end
    
    %% --- KINEMATICS INTEGRATION
    for i = 1:nSV
        % Longitudinal update (Equation 20-21)
        SV(i).y = SV(i).y + SV(i).v * dt;
        
        % Lateral position (FSM-based lane change with quintic smoothing)
        if SV(i).lc_active
            SV(i).lc_t = SV(i).lc_t + dt;
            s = smoothstep_quintic(min(SV(i).lc_t / SV(i).lc_T, 1.0));
            x_from = lane_center_of(SV(i).lane_from, lane_center_left, lane_center_middle, lane_center_right);
            x_to = lane_center_of(SV(i).lane_to, lane_center_left, lane_center_middle, lane_center_right);
            SV(i).x = (1-s)*x_from + s*x_to;
            
            if SV(i).lc_t >= SV(i).lc_T
                SV(i).lc_active = false;
                SV(i).lane_cur = SV(i).lane_to;
                SV(i).x = x_to;
                SV(i).lock_lane = true; % stabilization lock
                SV(i).ok_ctr = 0;
                fprintf('  🔄 t=%.1fs: SV%d completed lane change to %s\n', ...
                    t, i, char(SV(i).lane_cur));
                
                % *** RECORD OV POSITION WHEN SV REACHES MIDDLE LANE ***
                if SV(i).lane_cur == "MIDDLE"
                    middle_reach_events(end+1).sv_id = i;
                    middle_reach_events(end).t = t;
                    middle_reach_events(end).sv_y = SV(i).y;
                    middle_reach_events(end).ov_x = ov_x;
                    middle_reach_events(end).ov_y = ov_y;
                    
                    % Calculate relative position
                    if ov_y > SV(i).y
                        pos_rel = sprintf('OV %.1fm AHEAD', ov_y - SV(i).y);
                    else
                        pos_rel = sprintf('OV %.1fm BEHIND', SV(i).y - ov_y);
                    end
                    fprintf('  📍 SV%d reached MIDDLE: %s\n', i, pos_rel);
                end
            end
        else
            % Stay centered on current lane
            SV(i).x = lane_center_of(SV(i).lane_cur, lane_center_left, lane_center_middle, lane_center_right);
        end
        
        % Lane lock countdown
        if SV(i).lock_lane
            SV(i).ok_ctr = SV(i).ok_ctr + 1;
            if SV(i).ok_ctr >= HOLD_TICKS
                SV(i).lock_lane = false;
                SV(i).ok_ctr = 0;
            end
        end
        
        % Logging
        log(i).t(end+1,1) = t;
        log(i).x(end+1,1) = SV(i).x;
        log(i).y(end+1,1) = SV(i).y;
        log(i).v(end+1,1) = SV(i).v;
        log(i).theta(end+1,1) = SV(i).theta;
        risk_now = crpf(SV(i).x, SV(i).y, ov_x, ov_y, hypot(ov_vx, ov_vy), 0, 0, ...
            m_O, size_O, kappa_O, CRPF_G, CRPF_ZETA, PSEUDO_EPS, PSEUDO_RHO);
        log(i).riskOV(end+1,1) = risk_now;
        log(i).mode(end+1,1) = SV(i).mode;
        log(i).lane(end+1,1) = SV(i).lane_cur;
        
        if SV(i).preced > 0
            j = SV(i).preced;
            log(i).gap(end+1,1) = SV(j).y - SV(i).y;
            log(i).relv(end+1,1) = SV(j).v - SV(i).v;
        else
            log(i).gap(end+1,1) = NaN;
            log(i).relv(end+1,1) = NaN;
        end
    end
    
    % OV logging
    logOV.t(end+1,1) = t;
    logOV.x(end+1,1) = ov_x;
    logOV.y(end+1,1) = ov_y;
    logOV.vy(end+1,1) = ov_vy;
    
    % Collision detection
    for i = 1:nSV
        if hypot(SV(i).x - ov_x, SV(i).y - ov_y) <= Rsum
            collision = true;
            break;
        end
        for j = i+1:nSV
            if hypot(SV(i).x - SV(j).x, SV(i).y - SV(j).y) <= 2*Rsum
                collision = true;
                break;
            end
        end
    end
    
    % Console output (every 25 steps)
    if mod(k, 25) == 1 || k == 1
        fprintf('\n%-6s %-4s %-10s %-8s %9s %9s %7s\n', ...
            't[s]', 'ID', 'MODE', 'LANE', 'x[m]', 'y[m]', 'v[m/s]');
        fprintf('%s\n', repmat('-', 1, 64));
    end
    if mod(k, 25) == 1
        for ii = 1:nSV
            fprintf('%-6.1f %-4s %-10s %-8s %9.2f %9.2f %7.2f\n', ...
                t, sprintf('SV%d', ii), char(SV(ii).mode), char(SV(ii).lane_cur), ...
                SV(ii).x, SV(ii).y, SV(ii).v);
        end
        fprintf('%-6.1f %-4s %-10s %-8s %9.2f %9.2f %7.2f\n\n', ...
            t, 'OV', '-', 'MIDDLE', ov_x, ov_y, ov_vy);
    end
    
    if collision
        fprintf('\n⚠️  COLLISION DETECTED at t=%.2f s\n', t);
        break;
    end
end

%% --- VISUALIZATION

% 1. CRPF Heatmap at critical moment (when SV1 starts LEFT→MIDDLE)
first_lc_times = nan(nSV, 1);
for i = 1:nSV
    li = string(log(i).lane);
    idx = find(li(2:end) ~= "LEFT" & li(1:end-1) == "LEFT", 1, 'first');
    if ~isempty(idx)
        first_lc_times(i) = log(i).t(idx+1);
    end
end

% Filter out NaN values before taking min
valid_times = first_lc_times(~isnan(first_lc_times));
if ~isempty(valid_times)
    anchor_t = min(valid_times);
    [~, kA] = min(abs(logOV.t - anchor_t));
    anchor_ovx = logOV.x(kA);
    anchor_ovy = logOV.y(kA);
else
    anchor_ovx = logOV.x(1);
    anchor_ovy = logOV.y(1);
end

allY = [logOV.y; vertcat(log(:).y)];
ymin = floor(min(allY)/10)*10 - 20;
ymax = ceil(max(allY)/10)*10 + 20;

gx = linspace(-10, 10, 241);
gy = linspace(ymin, ymax, 301);
[GX, GY] = meshgrid(gx, gy);
v_obs_mag = hypot(ov_vx, ov_vy);
CR = arrayfun(@(x,y) crpf(x, y, anchor_ovx, anchor_ovy, v_obs_mag, 0, 0, ...
    m_O, size_O, kappa_O, CRPF_G, CRPF_ZETA, PSEUDO_EPS, PSEUDO_RHO), GX, GY);

figure('Color', 'w', 'Position', [100 100 1000 700]);
hold on;
imagesc(gx, gy, log10(CR + 1e-9));
axis xy;
colorbar;
title('CRPF Heatmap @ OV pose when SV1 starts LEFT→MIDDLE', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('X [m]', 'FontSize', 11);
ylabel('Y [m]', 'FontSize', 11);

% Contours
[~, hC] = contour(gx, gy, log10(CR + 1e-9), 12, 'LineWidth', 0.5);
set(hC, 'LineColor', [0.85 0.85 0.85]);

% Lane markers
xline(lane_center_left, 'k--', 'LineWidth', 1.5);
xline(lane_center_middle, 'k--', 'LineWidth', 1.5);
xline(lane_center_right, 'k--', 'LineWidth', 1.5);
text(lane_center_left, ymax-3, 'LEFT', 'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold');
text(lane_center_middle, ymax-3, 'MIDDLE', 'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold');
text(lane_center_right, ymax-3, 'RIGHT', 'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold');

% Trajectories
plot(logOV.x, logOV.y, 'k-', 'LineWidth', 2.5, 'DisplayName', 'OV');
for i = 1:nSV
    plot(log(i).x, log(i).y, '-', 'Color', SV(i).color, 'LineWidth', 2, ...
        'DisplayName', sprintf('SV%d', i));
    scatter(log(i).x(1), log(i).y(1), 50, SV(i).color, 'filled', 'MarkerEdgeColor', 'k');
end
scatter(logOV.x(1), logOV.y(1), 80, 'ko', 'filled', 'MarkerEdgeColor', 'w', 'LineWidth', 1.5);

% *** CROSS MARKS: OV position when each SV REACHES MIDDLE ***
for i = 1:length(middle_reach_events)
    scatter(middle_reach_events(i).ov_x, middle_reach_events(i).ov_y, 150, 'rx', 'LineWidth', 3.5);
    
    % Add label showing which SV and relative position
    if middle_reach_events(i).ov_y > middle_reach_events(i).sv_y
        label_text = sprintf('SV%d: OV ahead', middle_reach_events(i).sv_id);
    else
        label_text = sprintf('SV%d: OV behind', middle_reach_events(i).sv_id);
    end
    text(middle_reach_events(i).ov_x + 1.0, middle_reach_events(i).ov_y, label_text, ...
        'FontSize', 9, 'FontWeight', 'bold', 'Color', 'r');
end

xlim([-10 10]);
ylim([ymin ymax]);
grid on;
legend('Location', 'best');

% 2. Time-series plots
figure('Color', 'w', 'Position', [120 120 1200 800]);

tvec = log(1).t;

% Speed
subplot(4, 1, 1);
hold on;
for i = 1:nSV
    plot(tvec, log(i).v, 'Color', SV(i).color, 'LineWidth', 1.8, 'DisplayName', sprintf('SV%d', i));
end
plot(logOV.t, logOV.vy, 'k--', 'LineWidth', 2, 'DisplayName', 'OV');
yline(v_des_platoon, 'r:', 'LineWidth', 1.2, 'DisplayName', 'v_{des}');
ylabel('Speed [m/s]', 'FontSize', 10);
title('Longitudinal Speed', 'FontSize', 11, 'FontWeight', 'bold');
legend('Location', 'best');
grid on;

% Headway
subplot(4, 1, 2);
hold on;
for i = 1:nSV
    if SV(i).preced > 0
        plot(tvec, log(i).gap, 'Color', SV(i).color, 'LineWidth', 1.8, ...
            'DisplayName', sprintf('SV%d→SV%d', i, SV(i).preced));
    end
end
yline(d_headway_des, 'k--', 'LineWidth', 1.2, 'DisplayName', 'd_{des}');
ylabel('Gap [m]', 'FontSize', 10);
title('Headway to Predecessor', 'FontSize', 11, 'FontWeight', 'bold');
legend('Location', 'best');
grid on;

% CRPF Risk
subplot(4, 1, 3);
hold on;
for i = 1:nSV
    plot(tvec, log(i).riskOV, 'Color', SV(i).color, 'LineWidth', 1.8, ...
        'DisplayName', sprintf('SV%d', i));
end
yline(CR_YELLOW_LOW, 'g--', 'LineWidth', 1.2, 'DisplayName', 'Low');
yline(CR_YELLOW_HIGH, 'r--', 'LineWidth', 1.2, 'DisplayName', 'High');
ylabel('CRPF Risk', 'FontSize', 10);
title('Collision Risk (OV)', 'FontSize', 11, 'FontWeight', 'bold');
legend('Location', 'best');
grid on;

% Longitudinal position
subplot(4, 1, 4);
hold on;
for i = 1:nSV
    plot(log(i).t, log(i).y, 'LineWidth', 2, 'Color', SV(i).color, ...
        'DisplayName', sprintf('SV%d', i));
end
plot(logOV.t, logOV.y, 'k--', 'LineWidth', 2.5, 'DisplayName', 'OV');
ylabel('Y position [m]', 'FontSize', 10);
xlabel('Time [s]', 'FontSize', 10);
title('Longitudinal Position vs Time', 'FontSize', 11, 'FontWeight', 'bold');
legend('Location', 'best');
grid on;

% 3. X-Y Spatial Trajectory
figure('Color', 'w', 'Position', [140 140 900 700]);
hold on;
grid on;

% Lane backgrounds
fill([lane_center_left-lane_width/2, lane_center_left+lane_width/2, ...
      lane_center_left+lane_width/2, lane_center_left-lane_width/2], ...
     [ymin, ymin, ymax, ymax], [0.9 0.95 1], 'EdgeColor', 'none', 'FaceAlpha', 0.3);
fill([lane_center_middle-lane_width/2, lane_center_middle+lane_width/2, ...
      lane_center_middle+lane_width/2, lane_center_middle-lane_width/2], ...
     [ymin, ymin, ymax, ymax], [1 0.95 0.9], 'EdgeColor', 'none', 'FaceAlpha', 0.3);
fill([lane_center_right-lane_width/2, lane_center_right+lane_width/2, ...
      lane_center_right+lane_width/2, lane_center_right-lane_width/2], ...
     [ymin, ymin, ymax, ymax], [0.95 1 0.9], 'EdgeColor', 'none', 'FaceAlpha', 0.3);

% Lane lines
plot([lane_center_left, lane_center_left], [ymin, ymax], 'k--', 'LineWidth', 1.2);
plot([lane_center_middle, lane_center_middle], [ymin, ymax], 'k--', 'LineWidth', 1.2);
plot([lane_center_right, lane_center_right], [ymin, ymax], 'k--', 'LineWidth', 1.2);

% Trajectories
plot(logOV.x, logOV.y, 'k-', 'LineWidth', 3, 'DisplayName', 'OV');
for i = 1:nSV
    plot(log(i).x, log(i).y, '-', 'Color', SV(i).color, 'LineWidth', 2.5, ...
        'DisplayName', sprintf('SV%d', i));
    scatter(log(i).x(1), log(i).y(1), 80, SV(i).color, 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
    scatter(log(i).x(end), log(i).y(end), 80, SV(i).color, 'd', 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
end
scatter(logOV.x(1), logOV.y(1), 100, 'ko', 'filled', 'MarkerEdgeColor', 'w', 'LineWidth', 2);
scatter(logOV.x(end), logOV.y(end), 100, 'kd', 'filled', 'MarkerEdgeColor', 'w', 'LineWidth', 2);

% *** CROSS MARKS: OV position when each SV REACHES MIDDLE ***
for i = 1:length(middle_reach_events)
    scatter(middle_reach_events(i).ov_x, middle_reach_events(i).ov_y, 200, 'rx', 'LineWidth', 4);
    
    % Add connecting line from SV position to OV position
    sv_x = lane_center_middle;  % SV is at MIDDLE lane center
    sv_y = middle_reach_events(i).sv_y;
    plot([sv_x, middle_reach_events(i).ov_x], [sv_y, middle_reach_events(i).ov_y], ...
        'r:', 'LineWidth', 1.5);
    
    % Label
    if middle_reach_events(i).ov_y > middle_reach_events(i).sv_y
        label_text = sprintf('SV%d crosses\n(OV ahead)', middle_reach_events(i).sv_id);
        y_offset = -8;
    else
        label_text = sprintf('SV%d crosses\n(OV behind)', middle_reach_events(i).sv_id);
        y_offset = +8;
    end
    text(middle_reach_events(i).ov_x + 1.2, middle_reach_events(i).ov_y + y_offset, label_text, ...
        'FontSize', 9, 'FontWeight', 'bold', 'Color', 'r', 'BackgroundColor', [1 1 1 0.7]);
end

% Labels
text(lane_center_left, ymax-5, 'LEFT', 'HorizontalAlignment', 'center', 'FontSize', 11, 'FontWeight', 'bold');
text(lane_center_middle, ymax-5, 'MIDDLE', 'HorizontalAlignment', 'center', 'FontSize', 11, 'FontWeight', 'bold');
text(lane_center_right, ymax-5, 'RIGHT', 'HorizontalAlignment', 'center', 'FontSize', 11, 'FontWeight', 'bold');

title('Spatial Trajectories: Red X = OV when SV reaches MIDDLE', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('X [m] (Lateral)', 'FontSize', 11);
ylabel('Y [m] (Longitudinal)', 'FontSize', 11);
legend('Location', 'best');
xlim([-8 8]);
ylim([ymin ymax]);

% Final Summary
fprintf('\n========== SIMULATION SUMMARY ==========\n');
fprintf('Total time: %.1f s\n', t);
fprintf('Collision detected: %s\n', ternary(collision, 'YES ⚠️', 'NO ✅'));

for i = 1:nSV
    fprintf('\nSV%d:\n', i);
    fprintf('  Final lane: %s\n', char(SV(i).lane_cur));
    fprintf('  Final mode: %s\n', char(SV(i).mode));
    fprintf('  Final speed: %.2f m/s\n', SV(i).v);
    fprintf('  Crossed to RIGHT: %s\n', ternary(SV(i).crossed_to_right, 'YES ✅', 'NO ❌'));
    if ~isnan(SV(i).split_time)
        fprintf('  Split time: %.2f s\n', SV(i).split_time);
    end
    if ~isnan(SV(i).merge_time)
        fprintf('  Merge time: %.2f s\n', SV(i).merge_time);
    end
end

fprintf('\n--- OV Position When SVs Reached MIDDLE Lane ---\n');
for i = 1:length(middle_reach_events)
    rel_dist = middle_reach_events(i).ov_y - middle_reach_events(i).sv_y;
    if rel_dist > 0
        pos_str = sprintf('%.1fm AHEAD', rel_dist);
    else
        pos_str = sprintf('%.1fm BEHIND', abs(rel_dist));
    end
    fprintf('t=%.1fs: SV%d reached MIDDLE @ y=%.1fm, OV @ y=%.1fm (%s)\n', ...
        middle_reach_events(i).t, middle_reach_events(i).sv_id, ...
        middle_reach_events(i).sv_y, middle_reach_events(i).ov_y, pos_str);
end
fprintf('========================================\n');

%% ===== HELPER FUNCTIONS =====

    function start_lane_change(idx, next_lane, cross_type)
        SV(idx).lane_from = SV(idx).lane_cur;
        SV(idx).lane_to = next_lane;
        SV(idx).lc_t = 0;
        SV(idx).lc_active = true;
        SV(idx).lane_target = next_lane;
        SV(idx).track = lane_track_of(next_lane, TRACK_LEFT, TRACK_MIDDLE, TRACK_RIGHT);
        
        if ~isempty(cross_type)
            fprintf('  🚗 t=%.1fs: SV%d starts %s → %s (crossing from %s of OV)\n', ...
                t, idx, char(SV(idx).lane_from), char(next_lane), cross_type);
        else
            fprintf('  🚗 t=%.1fs: SV%d starts %s → %s\n', ...
                t, idx, char(SV(idx).lane_from), char(next_lane));
        end
    end

    function xc = lane_center_of(lbl, xL, xM, xR)
        if lbl == "LEFT"
            xc = xL;
        elseif lbl == "MIDDLE"
            xc = xM;
        else
            xc = xR;
        end
    end

    function TR = lane_track_of(lbl, TL, TM, TRt)
        if lbl == "LEFT"
            TR = TL;
        elseif lbl == "MIDDLE"
            TR = TM;
        else
            TR = TRt;
        end
    end

    function s = smoothstep_quintic(u)
        u = max(0, min(1, u));
        s = 10*u^3 - 15*u^4 + 6*u^5;
    end

    function v = ternary(cond, a, b)
        if cond
            v = a;
        else
            v = b;
        end
    end

    function safe = can_change_lane_smart(sv_idx, target_lane, SV, ov_x, ov_y, ov_vy, ...
            target_x, y_ahead, y_behind, safety_radius, desired_headway)
        % Smart lane change check - allows following predecessor
        
        sv_y = SV(sv_idx).y;
        
        % Safety window
        y_front = sv_y + y_ahead;
        y_back = sv_y - y_behind;
        
        % Check OV conflict
        ov_in_target = abs(ov_x - target_x) < 2.0;
        if ov_in_target && (ov_y >= y_back) && (ov_y <= y_front)
            ov_y_future = ov_y + ov_vy * TAU_LC;
            if ov_y_future >= y_back && ov_y_future <= y_front
                safe = false;
                return;
            end
        end
        
        % Check other SVs (WITH EXCEPTION FOR PREDECESSOR)
        for j = 1:numel(SV)
            if j == sv_idx
                continue;
            end
            
            % Check if this is the predecessor
            is_predecessor = (SV(sv_idx).preced == j);
            
            sj_in_target = (abs(SV(j).x - target_x) < 2.0) || ...
                           (SV(j).lc_active && SV(j).lane_to == target_lane) || ...
                           (SV(j).lane_cur == target_lane);
            
            if sj_in_target
                % If it's the predecessor, allow following if gap is reasonable
                if is_predecessor
                    gap = SV(j).y - sv_y;
                    % Allow if gap is within acceptable platoon range
                    if gap > desired_headway - 8.0 && gap < desired_headway + 15.0
                        continue;  % Skip this check - allow following
                    end
                end
                
                % For non-predecessors or badly-spaced predecessors, enforce safety
                if (SV(j).y >= y_back) && (SV(j).y <= y_front)
                    safe = false;
                    return;
                end
                
                sj_y_future = SV(j).y + SV(j).v * TAU_LC;
                if sj_y_future >= y_back && sj_y_future <= y_front
                    safe = false;
                    return;
                end
            end
        end
        
        safe = true;
    end

    function alpha = gate_alpha(risk_now, rlow, rhigh)
        if risk_now <= rlow
            alpha = 0;
        elseif risk_now >= rhigh
            alpha = 1;
        else
            tau = (risk_now - rlow) / max(1e-12, (rhigh - rlow));
            alpha = tau^2 * (3 - 2*tau);
        end
    end

    % CRPF Implementation (Section III-B, Equation 11)
    function val = crpf(xs, ys, xo, yo, v_obs, ax, ay, m_obs, size_obs, kappa_obs, G, zeta, eps, rho)
        rd = pseudo_distance(xs, ys, xo, yo, v_obs, eps, rho);
        rd = max(rd, 2.0);
        M = virtual_mass(m_obs, v_obs);
        Tfac = type_factor(size_obs, kappa_obs);
        phi = accel_factor(ax, ay, xs, ys, xo, yo);
        val = G * M * Tfac * phi / (rd^zeta);
    end

    function rd = pseudo_distance(xs, ys, xo, yo, v_obs, eps, rho)
        dx = xs - xo;
        dy = ys - yo;
        termx = eps * dx * exp(-rho * max(v_obs, 0.0));
        termy = eps * dy;
        rd = hypot(termx, termy);
    end

    function phi = accel_factor(ax, ay, xs, ys, xo, yo)
        K = 5.0;
        a = [ax, ay];
        rd = [xs - xo, ys - yo];
        na = norm(a);
        nr = norm(rd);
        if na < 1e-9 || nr < 1e-9
            phi = 1.0;
            return;
        end
        cos_t = dot(a/na, rd/nr);
        denom = K - na*cos_t;
        denom = min(max(denom, 0.2), 10.0);
        phi = K / denom;
    end

    function M = virtual_mass(m, v)
        M = m * (1.566e-14 * (max(v, 0.0)^6.687) + 0.3354);
    end

    function Tfac = type_factor(size, kappa)
        size_star = 4.0;
        kappa_star = 1.0;
        w1 = 1.0;
        w2 = 1.0;
        Tfac = (size / max(size_star, 1e-6))^w1 * (kappa / max(kappa_star, 1e-6))^w2;
    end

    function [collide, ttc] = will_collide(p_rel, v_rel, R, horizon)
        v2 = dot(v_rel, v_rel);
        if v2 < 1e-12
            collide = (norm(p_rel) <= R);
            ttc = inf;
            return;
        end
        
        tstar = -dot(p_rel, v_rel) / v2;
        tstar = min(max(tstar, 0.0), horizon);
        dmin = norm(p_rel + tstar*v_rel);
        ttc = inf;
        
        if dot(p_rel, v_rel) < 0
            a = v2;
            b = 2 * dot(p_rel, v_rel);
            c = dot(p_rel, p_rel) - R^2;
            disc = b*b - 4*a*c;
            if disc >= 0
                r1 = (-b - sqrt(disc)) / (2*a);
                r2 = (-b + sqrt(disc)) / (2*a);
                roots = [r1, r2];
                roots = roots(roots >= 0);
                if ~isempty(roots)
                    ttc = min(roots);
                end
            end
        end
        
        collide = (dmin <= R);
    end

end