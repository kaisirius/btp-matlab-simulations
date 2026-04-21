function simulate_part2()
% Blue-style zoom (v3) — MATLAB port
% - Y-axis shown from -50 to 500 m with ticks every 20 m
% - OV starting Y = +10 m
% - Adds TURN2 markers where SV merges back to the center lane
% Outputs: trajectory_zoom_blue.png

%% ---- params (kept identical) ----
DT = 0.1;
T_END = 40.0;
N = round(T_END/DT);

LANE_CENTER_X   = 0.0;
LANE_HALF_WIDTH = 4.0;
ROAD_LEFT  = LANE_CENTER_X + LANE_HALF_WIDTH; %#ok<NASGU>
ROAD_RIGHT = LANE_CENTER_X - LANE_HALF_WIDTH; %#ok<NASGU>

SV_x0     = 0.0;
SV_y0     = -26.0;
SV_vy0    = 12.0;
SV_theta0 = deg2rad(90.0); %#ok<NASGU>

OV_x0    = 0.0;
OV_y0    = 10.0;
OV_speed = 7.0;
OV_theta = deg2rad(90.0);
OV_vx    = OV_speed*cos(OV_theta);
OV_vy    = OV_speed*sin(OV_theta);
OV_MASS  = 1600.0;

VY_CRUISE = 14.0;
VY_PASS   = 20.0;

X_PASS        = 3.5;
TTC_THRESH    = 2.0;
AHEAD_GAP     = 10.0;
MIN_PASS_TIME = 1.6;
CRPF_LOW_MAX  = 1.6;

TAU_LC = 2.0;
TAU_MB = 2.0;

G    = 0.5;
zeta = 1.2;
rho  = 0.02;
epsY = 1.2;

%% ---- storage ----
sv_x = zeros(N,1); sv_y = zeros(N,1); sv_vy = zeros(N,1);
ov_x = zeros(N,1); ov_y = zeros(N,1);
phase_log = zeros(N,1,'int16'); % 0=follow,1=lc,2=pass,3=merge,4=done

sv_x(1)  = SV_x0;  sv_y(1)  = SV_y0;  sv_vy(1) = SV_vy0;
ov_x(1)  = OV_x0;  ov_y(1)  = OV_y0;

phase      = "follow";
phase_id   = struct('follow',0,'lc',1,'pass',2,'merge',3,'done',4);
t_phase    = 0.0;
turn_index = [];      % first lane-change start index
turn2_index = [];     % first return-to-center index
t_in_pass  = 0.0;

%% ---- simulate ----
for k = 2:N
    t = (k-1)*DT;

    % OV motion
    ov_y(k) = ov_y(k-1) + OV_vy*DT;
    ov_x(k) = OV_x0;

    same_lane = (abs(sv_x(k-1) - LANE_CENTER_X) < 0.5) && any(phase == ["follow","lc"]);
    if phase == "follow"
        if ttc_y(sv_y(k-1), sv_vy(k-1), ov_y(k-1), OV_vy, same_lane) < TTC_THRESH
            phase = "lc"; t_phase = 0.0; turn_index = k;
        end
    end

    if phase == "lc" && t_phase >= TAU_LC
        phase = "pass"; t_phase = 0.0; t_in_pass = 0.0;
    end

    if phase == "pass"
        t_in_pass = t_in_pass + DT;
        E_now = crpf_mag(sv_x(k-1), sv_y(k-1), ov_x(k-1), ov_y(k-1), OV_speed, G, zeta, rho, epsY, OV_MASS);
        ahead = (sv_y(k-1) - ov_y(k-1)) >= AHEAD_GAP;
        long_enough = (t_in_pass >= MIN_PASS_TIME);
        low_risk = (E_now <= CRPF_LOW_MAX);
        if (ahead || long_enough) && low_risk
            phase = "merge"; t_phase = 0.0;
        end
    end

    if phase == "merge"
        % detect first return to center within tolerance -> mark turn2
        if isempty(turn2_index) && abs(sv_x(k-1) - LANE_CENTER_X) < 0.05
            turn2_index = k-1;
        end
        if t_phase >= TAU_MB
            phase = "done"; t_phase = 0.0;
            if isempty(turn2_index)
                turn2_index = k; % fallback if tolerance not hit exactly
            end
        end
    end

    if phase == "done" && isempty(turn2_index)
        turn2_index = k;
    end

    % lateral reference
    switch phase
        case "follow"
            x_ref = LANE_CENTER_X;
        case "lc"
            x_ref = LANE_CENTER_X + (X_PASS - LANE_CENTER_X)*quintic_s(t_phase, TAU_LC);
        case "pass"
            x_ref = X_PASS;
        case "merge"
            x_ref = X_PASS + (LANE_CENTER_X - X_PASS)*quintic_s(t_phase, TAU_MB);
        otherwise
            x_ref = LANE_CENTER_X;
    end

    % longitudinal reference
    if any(phase == ["pass","merge"])
        vy_ref = VY_PASS;
    else
        vy_ref = VY_CRUISE;
    end

    sv_x(k)  = x_ref;
    % first-order lag to vy_ref with time-constant ≈ 1/1.6 s
    sv_vy(k) = sv_vy(k-1) + (vy_ref - sv_vy(k-1))*(1 - exp(-DT*1.6));
    sv_y(k)  = sv_y(k-1) + sv_vy(k)*DT;

    phase_log(k) = phase_id.(char(phase));
    t_phase = t_phase + DT;

    % early stop if we leave plotting window like the Python
    if sv_y(k) > 500
        sv_x  = sv_x(1:k);
        sv_y  = sv_y(1:k);
        sv_vy = sv_vy(1:k);
        ov_x  = ov_x(1:k);
        ov_y  = ov_y(1:k);
        phase_log = phase_log(1:k);
        N = k;
        break
    end
end

if isempty(turn_index)
    turn_index = floor(0.25 * N);
end

if isempty(turn2_index)
    idxs = find(phase_log==4 & abs(sv_x - LANE_CENTER_X) < 0.05, 1, 'first');
    if ~isempty(idxs), turn2_index = idxs; else, turn2_index = N; end
end

xo = ov_x(turn_index);
yo = ov_y(turn_index);

%% ---- CRPF grid at turn instant (like Python) ----
gx = linspace(-12, 12, 241);
gy = linspace(-50, 500, 1001);
[GX, GY] = meshgrid(gx, gy);
Z = crpf_grid(GX, GY, xo, yo, OV_speed, G, zeta, rho, epsY, OV_MASS);

%% ---- plot ----
figure('Color','w','Position',[100 100 1260 680]); hold on;

% pcolormesh equivalent
imagesc(gx, gy, Z); set(gca,'YDir','normal');
colormap(turbo);
cb = colorbar; ylabel(cb, 'log_{10}(|E_V|)');

% contour levels
Zmin = min(Z(:)); Zmax = max(Z(:));
levels = linspace(Zmin + 0.2*(Zmax - Zmin), Zmax, 14);
[~, hC] = contour(GX, GY, Z, levels, 'LineColor', [1 1 1], 'LineWidth', 0.7); %#ok<ASGLU>

% Paths
crimson = [220 20 60]/255;
orange  = [1.0 0.55 0.0];
plot(sv_x, sv_y, 'Color', crimson, 'LineWidth', 2.2, 'DisplayName','SV path');
plot(ov_x, ov_y, '--', 'Color', orange, 'LineWidth', 1.8, 'DisplayName','OV path');

% Markers: starts
scatter(sv_x(1), sv_y(1), 36, 'MarkerFaceColor', crimson, 'MarkerEdgeColor','k', 'DisplayName','SV start', 'LineWidth',0.8);
scatter(ov_x(1), ov_y(1), 36, 'MarkerFaceColor', orange,  'MarkerEdgeColor','k', 'DisplayName','OV start', 'LineWidth',0.8);

% turn1
scatter(sv_x(turn_index), sv_y(turn_index), 44, 'MarkerFaceColor', crimson, 'MarkerEdgeColor','k', 'DisplayName','SV @ turn', 'LineWidth',0.8);
scatter(ov_x(turn_index), ov_y(turn_index), 44, 'MarkerFaceColor', orange,  'MarkerEdgeColor','k', 'DisplayName','OV @ turn', 'LineWidth',0.8);

% turn2 (unfilled)
scatter(sv_x(turn2_index), sv_y(turn2_index), 44, 'o', 'MarkerEdgeColor','k', 'MarkerFaceColor', 'none', 'LineWidth',1.3, 'DisplayName','SV @ turn2');
scatter(ov_x(turn2_index), ov_y(turn2_index), 44, 'o', 'MarkerEdgeColor','k', 'MarkerFaceColor', 'none', 'LineWidth',1.3, 'DisplayName','OV @ turn2');

% Lane markers
xline(LANE_CENTER_X - LANE_HALF_WIDTH, ':', 'Color','k', 'LineWidth',1.4);
xline(LANE_CENTER_X + LANE_HALF_WIDTH, ':', 'Color','k', 'LineWidth',1.4);
for xl = [-4 4]
    xline(xl, ':', 'Color', [0 0 0], 'LineWidth',0.9);
end

xlim([-12 12]);
ylim([-50 500]);
yticks(-50:20:500);
xlabel('X [m]');
ylabel('Y [m]');
title(sprintf('CRPF at turn instant (t = %.1fs) + SV/OV Trajectories (zoomed main)', (turn_index-1)*DT));
legend('Location','northeast');

% Save at ~170 dpi
exportgraphics(gcf, 'trajectory_zoom_blue.png', 'Resolution', 170);

fprintf('Saved trajectory_zoom_blue.png\n');
fprintf('turn_index=%d, turn2_index=%d, t_turn2=%.2fs\n', turn_index, turn2_index, (turn2_index-1)*DT);
end

%% ---------- helpers ----------
function val = crpf_mag(xs, ys, xo, yo, vov, G, zeta, rho, epsY, OV_MASS)
    rd = pseudo_distance(xs, ys, xo, yo, vov, rho, epsY);
    M  = virtual_mass(OV_MASS, vov);
    val = (G*M) / (rd.^zeta);
end

function rd = pseudo_distance(xs, ys, xo, yo, vov, rho, epsY)
    dx = xs - xo;
    dy = ys - yo;
    scale = exp(-rho*max(vov,0.0));
    rd = hypot(dx, (abs(dy).^epsY)*scale) + 1e-9;
end

function M = virtual_mass(m, v)
    M = m*(1.566e-14*(max(v,0.0)^6.687) + 0.3354);
end

function out = ttc_y(svy, svvy, ovy, ovvy, same_lane)
    if ~same_lane, out = 1e9; return; end
    relv = svvy - ovvy;
    rely = ovy - svy;
    if relv <= 0 || rely <= 0
        out = 1e9;
    else
        out = rely/relv;
    end
end

function s = quintic_s(t, T)
    tau = max(0.0, min(1.0, t/T));
    s = 10*tau^3 - 15*tau^4 + 6*tau^5;
end

function Z = crpf_grid(GX, GY, xo, yo, speed, G, zeta, rho, epsY, OV_MASS)
    % vectorized version of Python crpf_grid
    Mconst = virtual_mass(OV_MASS, speed);
    scale  = exp(-0.02 * speed);
    dx = GX - xo;
    dy = abs(GY - yo);
    rd = sqrt(dx.^2 + (dy.^epsY).*(scale.^2)) + 1e-9;
    CR = (G*Mconst) ./ (rd.^zeta);
    Z = log10(CR + 1e-12);
end