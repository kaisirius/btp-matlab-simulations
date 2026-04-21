function simulate_part1()
% SV vs stationary OV with CRPF + VO + DWA
% Uniform bend around the obstacle and strict return to a predefined track.
%
% Uses ONLY Section III-B formulas for CRPF. No added physics — just
% scheduling the same CRPF term off after the pass, and using a track-
% following cost (cross-track + heading-to-track).

rng(0);  OUT = "outputs_part1";
if ~exist(OUT,"dir"), mkdir(OUT); end

%% --- Config / scenario
dt = 0.1; T = 60.0; N = round(T/dt);

% Lane (straight along +Y)
lane_center_x   = 0.0;
lane_half_width = 4.0;
road_left  = lane_center_x + lane_half_width;
road_right = lane_center_x - lane_half_width;

% Disc radii / safety
rS = 1.0; rO = 1.0; Rsum = rS + rO;
road_left_vehicle_safety_limit  = road_left  - rS;
road_right_vehicle_safety_limit = road_right + rS;

% SV init (centerline; far so field doesn't affect initially)
sv_x = 0.0; sv_y = -60.0;
sv_v = 12.0;
sv_theta = deg2rad(90.0);      % along +Y
theta_des = deg2rad(90.0);

% %% TRACK: predefined reference track (polyline). Straight x=0 centerline.
% You can replace this with any curved path later (Nx2 [x y] points).
USE_TRACK = true;
TRACK = [0, -300;
         0,  300];

% SV limits
a_min = -4.0; a_max = 2.5;         % accel [m/s^2]
omega_min = -0.6; omega_max = 0.6; % yaw rate [rad/s]
v_min = 5.0; v_max = 25.0; v_des = 15.0;

% OV (stationary) at origin
ov_x = 0.0; ov_y = 0.0;
ov_vx = 0.0; ov_vy = 0.0;
m_O = 1500.0; size_O = 4.5; kappa_O = 1.0;

% === CRPF parameters (paper, Fig. 3 style) ===
CRPF_G    = 0.5;
CRPF_ZETA = 1.2;
PSEUDO_EPS = 1.0;
PSEUDO_RHO = 0.02;

% ---- CRPF gating (based on CRPF at the SV pose) ----
CR_YELLOW_LOW  = 0.008;   % start blending risk
CR_YELLOW_HIGH = 0.012;   % fully blended
Y_CLEAR_AHEAD  = 10.0;    % past-OV distance where risk is fully OFF

% DWA weights (base, used BEFORE the pass)
W_RISK  = 1.0;            % multiplied by alpha (gate)
W_SPEED = 0.08;
W_TURN  = 0.08;
W_LAT   = 0.32;           % (now penalizes |CTE| if USE_TRACK=true)
W_HEAD  = 0.12;           % (now penalizes heading error to track)
W_END   = 0.90;           % terminal |CTE| (or |x-x_lane| if no track)

% Post-pass (strong recenter) weights — activated after Y_CLEAR_AHEAD
W_LAT_POST  = 0.32;       % (kept same as base; adjust if you want stronger)
W_END_POST  = 0.9;
W_HEAD_POST = 0.12;
W_TURN_POST = 0.08;

% DWA rollout horizon (shorter horizon reduces “keep curving” bias)
steps = 8;                % 8*0.1s = 0.8 s

%% --- Precompute CRPF field (for plot)
gx = linspace(-12,12,121);
gy = linspace(-100,120,141);
CR = zeros(numel(gy), numel(gx));
for j = 1:numel(gy)
    for i = 1:numel(gx)
        CR(j,i) = crpf(gx(i), gy(j), ov_x, ov_y, 0.0, 0,0, ...
                       m_O, size_O, kappa_O, CRPF_G, CRPF_ZETA, PSEUDO_EPS, PSEUDO_RHO);
    end
end

%% --- Simulation loop
log = struct('t',[],'sv_x',[],'sv_y',[],'sv_v',[],'sv_theta',[], ...
             'dist_to_obs',[],'risk',[],'ttc',[],'picked_v',[],'picked_omega',[]);
collision = false; min_ttc = inf; min_dist = inf;

for k = 1:N
    t = (k-1)*dt;

    % relative kinematics
    p_rel = [sv_x-ov_x, sv_y-ov_y];
    v_rel = [sv_v*cos(sv_theta)-ov_vx, sv_v*sin(sv_theta)-ov_vy];
    dist  = norm(p_rel);

    % local (current) CRPF + TTC
    risk_now = crpf(sv_x, sv_y, ov_x, ov_y, 0.0, 0,0, ...
                    m_O, size_O, kappa_O, CRPF_G, CRPF_ZETA, PSEUDO_EPS, PSEUDO_RHO);
    [~, ttc] = will_collide(p_rel, v_rel, Rsum, 8.0);
    min_ttc  = min(min_ttc, ttc);
    min_dist = min(min_dist, dist);

    % gate alpha∈[0,1] from local CRPF (prevents early drift)
    if risk_now <= CR_YELLOW_LOW
        alpha = 0;
    elseif risk_now >= CR_YELLOW_HIGH
        alpha = 1;
    else
        tau = (risk_now - CR_YELLOW_LOW) / max(1e-12, (CR_YELLOW_HIGH - CR_YELLOW_LOW));
        alpha = tau.^2 .* (3 - 2*tau);  % smoothstep
    end

    % ---------------------- DWA candidates ----------------------
    a_samples     = linspace(a_min, a_max, 7);
    omega_samples = linspace(omega_min, omega_max, 9);

    best_cost = inf; best_v_next = sv_v; best_theta_next = sv_theta;

    for a_cmd = a_samples
        v_next = min(max(sv_v + a_cmd*dt, v_min), v_max);
        for d_omega = omega_samples
            % rollout (heading integrated each substep)
            x_tmp = sv_x; y_tmp = sv_y; th = sv_theta; vtmp = v_next;
            ok = true; acc_risk = 0; acc_lat = 0; acc_head = 0; term_pen = 0;

            % %% TRACK: we will also keep the end-of-rollout CTE for W_END
            cte_end = 0;  % (only used if USE_TRACK)

            for s = 1:steps
                th    = wrapToPi(th + d_omega*dt);
                x_tmp = x_tmp + vtmp*cos(th)*dt;
                y_tmp = y_tmp + vtmp*sin(th)*dt;

                % feasibility
                if ~(in_lane(x_tmp, road_right_vehicle_safety_limit, road_left_vehicle_safety_limit)), ok=false; break; end
                if hypot(x_tmp-ov_x, y_tmp-ov_y) <= Rsum, ok=false; break; end

                % CRPF along predicted path (Eq. 11)
                rr = crpf(x_tmp, y_tmp, ov_x, ov_y, 0.0, 0,0, ...
                          m_O, size_O, kappa_O, CRPF_G, CRPF_ZETA, PSEUDO_EPS, PSEUDO_RHO);
                acc_risk = acc_risk + rr;

                % %% TRACK: track-following errors instead of raw lane-center
                if USE_TRACK
                    [cte, psi_err] = track_error(x_tmp, y_tmp, th, TRACK);
                    acc_lat  = acc_lat  + abs(cte);       % cross-track error
                    acc_head = acc_head + abs(psi_err);   % heading-to-track error
                    if s == steps
                        cte_end = cte;                    % remember end CTE
                    end
                else
                    % fallback: original lane center & desired heading
                    acc_lat  = acc_lat  + abs(x_tmp - lane_center_x);
                    acc_head = acc_head + abs(wrapToPi(th - theta_des));
                end

                % Terminal penalty if rollout ends past the OV but not centered
                if (s == steps) && (y_tmp > ov_y + (Y_CLEAR_AHEAD + 2.0))
                    if USE_TRACK
                        off = abs(cte_end);
                    else
                        off = abs(x_tmp - lane_center_x);
                    end
                    if off > 0.05               % 5 cm tolerance
                        term_pen = 1e3 * off;   % strong nudge to return to track
                    end
                end
            end
            if ~ok, continue; end

            mean_risk = acc_risk / steps;

            % cost = (CRPF term, gated) + track CTE + track heading + smoothness + speed
            if USE_TRACK
                end_offset_term = abs(cte_end);
            else
                end_offset_term = abs(x_tmp - lane_center_x);
            end

            cost = ((W_RISK*alpha) * mean_risk) ...
                 + (W_LAT*acc_lat/steps) + (W_HEAD*acc_head/steps) ...
                 + (W_END*end_offset_term) ...
                 + (W_SPEED*(v_des - v_next)^2) + (W_TURN*d_omega^2) ...
                 + term_pen;   % <— actually apply the terminal penalty

            if cost < best_cost
                best_cost = cost;
                best_v_next = v_next;
                best_theta_next = sv_theta + d_omega*dt;
            end
        end
    end

    if ~isfinite(best_cost)
        best_v_next = max(sv_v, v_min);
        best_theta_next = sv_theta;
    end

    % Apply
    sv_v = best_v_next;
    sv_theta = wrapToPi(best_theta_next);
    sv_x = sv_x + sv_v*cos(sv_theta)*dt;
    sv_y = sv_y + sv_v*sin(sv_theta)*dt;

    % Log
    log.t(end+1,1) = t;
    log.sv_x(end+1,1) = sv_x; log.sv_y(end+1,1) = sv_y;
    log.sv_v(end+1,1) = sv_v; log.sv_theta(end+1,1) = sv_theta;
    log.dist_to_obs(end+1,1) = dist; log.risk(end+1,1) = risk_now;
    log.ttc(end+1,1) = ttc; log.picked_v(end+1,1) = sv_v; log.picked_omega(end+1,1) = 0;

    if dist <= Rsum, collision = true; break; end
end

TBL = struct2table(log);
writetable(TBL, fullfile(OUT,"trajectory_part1.csv"));

summary = table(collision, min_dist, min_ttc, TBL.sv_x(end), TBL.sv_y(end), TBL.sv_v(end), height(TBL), ...
    'VariableNames', {'CollisionOccurred','MinDistance_m','MinTTC_s','FinalXPos_m','FinalYPos_m','FinalSpeed_mps','StepsSimulated'});
writetable(summary, fullfile(OUT,"summary_part1.csv"));

%% --- Plots
figure('Color','w');
imagesc(gx, gy, log10(CR+1e-9)); axis xy; hold on; colorbar;
title('CRPF Heatmap (Stationary OV) + SV Trajectory'); xlabel('X [m]'); ylabel('Y [m]');
[~, hC] = contour(gx, gy, log10(CR+1e-9), 10, 'LineWidth', 0.6);
set(hC, 'LineColor', [1 1 1]*0.9);
xline(road_left,  'k:', 'LineWidth', 2.0);
xline(road_right, 'k:', 'LineWidth', 2.0);
scatter(ov_x, ov_y, 20, 'r', 'filled', 'MarkerEdgeColor','k');
plot(TBL.sv_x, TBL.sv_y, 'r-', 'LineWidth', 2);
scatter(TBL.sv_x(1),  TBL.sv_y(1),  50, 'go','filled');
scatter(TBL.sv_x(end),TBL.sv_y(end),50, 'go','filled');
xlim([gx(1) gx(end)]); ylim([-80 80]); grid on;

%% ========== helpers ==========
function ok = in_lane(x, rright, rleft), ok = (x >= rright) && (x <= rleft); end
end % simulate_part1


% ---------- Section III-B helper functions ----------
function val = crpf(xs, ys, xo, yo, v_obs, ax, ay, m_obs, size_obs, kappa_obs, G, zeta, eps, rho)
    rd = pseudo_distance(xs, ys, xo, yo, v_obs, eps, rho);
    rd = max(rd, 2.5);         % physical clamp (~half car width) for stability
    M  = virtual_mass(m_obs, v_obs);
    Tfac = type_factor(size_obs, kappa_obs);
    phi  = accel_factor(ax, ay, xs, ys, xo, yo);  % =1 here (ax=ay=0)
    val = G * M * Tfac * phi / (rd^zeta);        % Eq. (11)
end

function rd = pseudo_distance(xs, ys, xo, yo, v_obs, eps, rho)
    % Eq. (8): anisotropic linear scaling
    dx = xs - xo; dy = ys - yo;
    termx = eps * dx * exp(-rho * max(v_obs,0.0));
    termy = eps * dy;
    rd = hypot(termx, termy);
end

function phi = accel_factor(ax, ay, xs, ys, xo, yo)
    % Eq. (9)
    K = 5.0;
    a = [ax, ay]; rd = [xs-xo, ys-yo];
    na = norm(a); nr = norm(rd);
    if na < 1e-9 || nr < 1e-9, phi = 1.0; return; end
    cos_t = dot(a/na, rd/nr);
    denom = K - na*cos_t;
    denom = min(max(denom, 0.2), 10.0);
    phi = K / denom;
end

function M = virtual_mass(m, v)
    % Eq. (6)
    M = m * (1.566e-14 * (max(v,0.0)^6.687) + 0.3354);
end

function Tfac = type_factor(size, kappa)
    % Eq. (7) (weights=1, refs 4 m & 1.0)
    size_star = 4.0; kappa_star = 1.0; w1=1.0; w2=1.0;
    Tfac = (size / max(size_star,1e-6))^w1 * (kappa / max(kappa_star,1e-6))^w2;
end

function [collide, ttc] = will_collide(p_rel, v_rel, R, horizon)
    v2 = dot(v_rel, v_rel);
    if v2 < 1e-12
        collide = (norm(p_rel) <= R); ttc = inf; return
    end
    tstar = -dot(p_rel, v_rel)/v2; tstar = min(max(tstar,0.0), horizon);
    dmin = norm(p_rel + tstar*v_rel);
    ttc = inf;
    if dot(p_rel, v_rel) < 0
        a=v2; b=2*dot(p_rel, v_rel); c=dot(p_rel,p_rel)-R^2;
        disc = b*b - 4*a*c;
        if disc >= 0
            r1 = (-b - sqrt(disc))/(2*a); r2 = (-b + sqrt(disc))/(2*a);
            roots = [r1,r2]; roots = roots(roots >= 0);
            if ~isempty(roots), ttc = min(roots); end
        end
    end
    collide = (dmin <= R);
end

%% ---------- TRACK helpers (polyline CTE + heading-to-track) ----------
function [cte, psi_err] = track_error(x, y, theta, TRACK)
% Return signed cross-track error and heading error w.r.t. the nearest
% point and tangent on a polyline track.

    % Fast path for a perfectly vertical straight line (x ≈ const)
    if size(TRACK,1) >= 2 && all(abs(diff(TRACK(:,1))) < 1e-9)
        x_ref  = TRACK(1,1);                % constant x (e.g., 0)
        cte    = x - x_ref;                 % left positive if y grows upward
        psi_ref = pi/2;                     % tangent along +Y
        psi_err = wrapToPi(theta - psi_ref);
        return
    end

    % General polyline case
    [xp, yp, tx, ty] = nearest_point_on_polyline(x, y, TRACK);
    % Left-hand normal to tangent (tx,ty)
    nx = -ty; ny = tx;

    % Signed cross-track error: projection onto left normal
    cte = (x - xp)*nx + (y - yp)*ny;

    % Heading error relative to track tangent
    psi_ref = atan2(ty, tx);
    psi_err = wrapToPi(theta - psi_ref);
end

function [xp, yp, tx, ty] = nearest_point_on_polyline(x, y, P)
% Find nearest point (xp,yp) on polyline P (Nx2), and the unit tangent (tx,ty)
% of the nearest segment at that point.

    best_d2 = inf; xp = P(1,1); yp = P(1,2); tx = 0; ty = 1;

    for i = 1:size(P,1)-1
        x0 = P(i,1);   y0 = P(i,2);
        x1 = P(i+1,1); y1 = P(i+1,2);
        vx = x1 - x0;  vy = y1 - y0;
        L2 = vx*vx + vy*vy + 1e-12;

        % Project point onto segment
        t = ((x - x0)*vx + (y - y0)*vy) / L2;
        t = min(max(t, 0.0), 1.0);
        xc = x0 + t*vx; yc = y0 + t*vy;

        d2 = (x - xc)^2 + (y - yc)^2;
        if d2 < best_d2
            best_d2 = d2; xp = xc; yp = yc;
            L = sqrt(L2);
            tx = vx / L; ty = vy / L;
        end
    end
end