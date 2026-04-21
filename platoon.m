function platoon()
% 3-vehicle platoon vs a STATIONARY OV with CRPF + VO + DWA (longitudinal only).
% Lateral is lane-restricted: MAIN (x=0), LEFT (x=-4), RIGHT (x=+4).
% Lane changes use a smooth quintic, and SVs may NOT leave lane centers/corridors.
% RULE: Overtaking allowed from RIGHT lane ONLY. Left lane is forbidden for overtaking.

rng(0); OUT = "outputs_platoon_static";
if ~exist(OUT,"dir"), mkdir(OUT); end

%% --- Global config
dt = 0.10; T = 60.0; N = round(T/dt);

% Road / lanes (symmetric)
lane_center_main  =  0.0;
lane_center_left  = -4.0;
lane_center_right = +4.0;

% Safety / radii
rS = 1.0; rO = 1.0; Rsum = rS + rO;

% Tracks (only for drawing)
TRACK_MAIN  = [lane_center_main,   -500; lane_center_main,   +500];
TRACK_LEFT  = [lane_center_left,   -500; lane_center_left,   +500];
TRACK_RIGHT = [lane_center_right,  -500; lane_center_right,  +500];

% Platoon targets
v_des_platoon   = 15.0;   % m/s (SVs faster than OV)
d_headway_des   = 18.0;   % m
merge_headway_tol = 3.5;  % m

% Longitudinal limits (no steering search; omega=0)
a_min = -4.0; a_max = +2.5;
v_min = 5.0;  v_max = 25.0;

% --- overtaking helpers (speed floor while behind on a passing lane)
V_PASS_MARGIN   = 4.5;   % m/s faster than OV while behind
Y_BEHIND_PAD    = 2.0;   % "behind" if SV.y <= OV.y + this

% OV (STATIONARY on MAIN)
ov_x = lane_center_main;
ov_y = +20.0;
ov_vx = 0.0;
ov_vy = 0.0;   % stationary
m_O = 1500.0; size_O = 4.5; kappa_O = 1.0;

% === CRPF ===
CRPF_G    = 0.5;
CRPF_ZETA = 1.2;
PSEUDO_EPS = 1.0;
PSEUDO_RHO = 0.02;

% Risk gating / clear-ahead
CR_YELLOW_LOW  = 0.008;
CR_YELLOW_HIGH = 0.012;
Y_CLEAR_AHEAD  = 10.0;

% DWA horizon (longitudinal only)
steps = 8;   % 0.8 s

% Longitudinal DWA weights
W_RISK  = 1.0;
W_SPEED = 0.08;

% Exact recenter parameters
HOLD_TICKS = 5;

% Lane-change timing (quintic)
TAU_LC = 2.0;          % duration of lane-change (s)
EPS_CTE = 0.03;        % used for recenter lock counter
EPS_HEAD = deg2rad(2);

% ----- console table config (place before for k=1:N) -----
PRINT_EVERY   = 0.5;   % seconds between prints
next_print    = 0.0;
printed_header = false;

%% --- 3 SVs (changed from 4)
nSV = 3;
SV = repmat(struct('x',0,'y',0,'v',v_des_platoon,'theta',pi/2, ...
    'mode',"PLATOON",'lane_target',"MAIN",'track',TRACK_MAIN, ...
    'preced',0,'color',[0 0 0],'split_time',NaN,'merge_time',NaN, ...
    'recenter',false,'lock_main',false,'ok_ctr',0, ...
    'lane_cur',"MAIN",'lc_active',false,'lc_t',0,'lc_T',TAU_LC, ...
    'lane_from',"MAIN",'lane_to',"MAIN"), nSV,1);

lead_y = -40.0; gap0 = d_headway_des;
for i=1:nSV
    SV(i).x = lane_center_main;
    SV(i).y = lead_y - (i-1)*gap0;
    SV(i).v = v_des_platoon;
    SV(i).theta = pi/2;
    SV(i).preced = (i>1)*(i-1);
end
% Explicit palette: SV1=blue, SV2=red, SV3=orange
palette = [
    0.0000 0.4470 0.7410  % blue
    0.6350 0.0780 0.1840  % red
    0.8500 0.3250 0.0980  % orange
];
linestyles = {'-','-','--'};
for i=1:nSV
    SV(i).color = palette(i,:);
    SV(i).style = linestyles{i};
end


% Logs
log(nSV)=struct('t',[],'x',[],'y',[],'v',[],'theta',[],'riskOV',[],'gap',[],'relv',[],'mode',strings(0,1),'lane',strings(0,1));
for i=1:nSV, log(i)=struct('t',[],'x',[],'y',[],'v',[],'theta',[],'riskOV',[],'gap',[],'relv',[],'mode',strings(0,1),'lane',strings(0,1)); end
logOV = struct('t',[],'x',[],'y',[],'vy',[]);

collision = false;

%% --- Sim loop
for k=1:N
  t = (k-1)*dt;

  % OV kinematics (stationary)
  ov_x = ov_x + ov_vx*dt; %#ok<NASGU>
  ov_y = ov_y + ov_vy*dt; %#ok<NASGU>

  % --- SIDE DECISION: RIGHT lane ONLY for overtaking. Left lane is forbidden.
  for i=1:nSV
      % Honor an ongoing lane-change
      if SV(i).lc_active
          SV(i).lane_target = SV(i).lane_to;
          SV(i).track = lane_track_of(SV(i).lane_to, TRACK_MAIN, TRACK_LEFT, TRACK_RIGHT);
          continue
      end

      % If we're in lock_main (recenter phase), keep MAIN and skip side-choice
      if SV(i).lock_main
          SV(i).lane_target = "MAIN";
          SV(i).track       = TRACK_MAIN;
          continue
      end

      dyOV = ov_y - SV(i).y;
      need_consider = (dyOV > 0) && (dyOV < 40);

      if need_consider
          center_conflict = (SV(i).lane_cur=="MAIN");
          if center_conflict
              y0 = SV(i).y - 6; y1 = SV(i).y + 12;
              % Only check RIGHT lane for overtaking (LEFT lane is forbidden)
              freeR = lane_window_free(lane_center_right, y0, y1, ov_x, ov_y, SV, i);

              if freeR
                  % Right lane is free -> move to RIGHT
                  start_lane_change(i, "RIGHT");
              else
                  % Right lane is busy (other SVs ahead) -> stay on MAIN and slow down
                  % (No left lane fallback - just wait on MAIN)
              end
          else
              % Already off MAIN on RIGHT? if far past OV, head back to MAIN
              if (SV(i).lane_cur~="MAIN") && (SV(i).y > (ov_y + Y_CLEAR_AHEAD + 5.0))
                  start_lane_change(i, "MAIN");
                  SV(i).recenter  = true;
                  SV(i).lock_main = true;   % freeze side-choice while recentering
              end
          end
      else
          if (SV(i).lane_cur~="MAIN") && (SV(i).y > (ov_y + Y_CLEAR_AHEAD + 5.0))
              start_lane_change(i, "MAIN");
              SV(i).recenter  = true;
              SV(i).lock_main = true;
          end
      end
  end

  % --- LONGITUDINAL DWA for each SV (omega=0; lateral is FSM)
  for i=1:nSV
      vxi = 0;
      vyi = SV(i).v;

      % TTC/Risk w.r.t. OV (stationary)
      p_rel_OV = [SV(i).x-ov_x, SV(i).y-ov_y];
      v_rel_OV = [vxi-ov_vx,     vyi-ov_vy];
      risk_OV  = crpf(SV(i).x, SV(i).y, ov_x, ov_y, 0.0, 0,0, ...
                      m_O, size_O, kappa_O, CRPF_G, CRPF_ZETA, PSEUDO_EPS, PSEUDO_RHO);
      [~, ttcOV] = will_collide(p_rel_OV, v_rel_OV, Rsum, 8.0);

      % TTC to predecessor (1D along Y)
      if SV(i).preced>0
         j = SV(i).preced;
         p_rel_P = [0, SV(i).y - SV(j).y];
         v_rel_P = [0, SV(i).v - SV(j).v];
         [~, ttcP] = will_collide(p_rel_P, v_rel_P, 2*Rsum, 8.0);
      else
         ttcP = inf;
      end

      % Split conditions (early & safe)
      need_split = (ttcOV < 2.2) || (ttcP < 1.5) || (risk_OV >= CR_YELLOW_HIGH);
      if SV(i).mode=="PLATOON" && need_split
          SV(i).mode = "CRUISE"; SV(i).split_time = t;
      end

      % Merge back (only if predecessor exists and already on MAIN and we've ordered MAIN)
      if SV(i).mode=="CRUISE" && SV(i).preced>0 && ~SV(i).lc_active && SV(i).lane_target=="MAIN"
          j = SV(i).preced;
          gap = (SV(j).y - SV(i).y);
          can_merge = (risk_OV <= CR_YELLOW_LOW) ...
                   && (gap > d_headway_des-merge_headway_tol) ...
                   && (gap < d_headway_des+2*merge_headway_tol) ...
                   && (SV(i).lane_cur=="MAIN"); % already arrived MAIN
          if can_merge
              SV(i).mode = "PLATOON"; SV(i).merge_time = t;
          end
      end

      % DWA candidates (acceleration only)
      a_samples = linspace(a_min, a_max, 7);
      alpha = gate_alpha(risk_OV, CR_YELLOW_LOW, CR_YELLOW_HIGH);

      best_cost = inf; best_v_next = SV(i).v;

      for a_cmd = a_samples
          v_next = min(max(SV(i).v + a_cmd*dt, v_min), v_max);

          % rollout purely along Y; lateral handled separately
          y_tmp = SV(i).y; acc_risk=0;
          for s=1:steps
              y_tmp = y_tmp + v_next*dt;   % theta = pi/2
              rr = crpf(SV(i).x, y_tmp, ov_x, ov_y, 0.0, 0,0, ...
                        m_O, size_O, kappa_O, CRPF_G, CRPF_ZETA, PSEUDO_EPS, PSEUDO_RHO);
              acc_risk = acc_risk + rr;
          end
          mean_risk = acc_risk/steps;

          cost = ((W_RISK*alpha) * mean_risk) ...
               + (W_SPEED*(v_des_platoon - v_next)^2);

          if cost < best_cost
              best_cost   = cost;
              best_v_next = v_next;
          end
      end

      % Overtake floor if behind and on RIGHT passing lane ONLY
      % (LEFT lane is forbidden so no floor boost if somehow on left)
      is_passing_lane = (SV(i).lane_cur=="RIGHT") || (SV(i).lc_active && SV(i).lane_to=="RIGHT");
      still_behind    = (SV(i).y <= ov_y + Y_BEHIND_PAD);
      if is_passing_lane && still_behind
          v_floor = min(v_max, ov_vy + V_PASS_MARGIN); % ov_vy==0 -> v_floor=V_PASS_MARGIN
          best_v_next = max(best_v_next, v_floor);
      end

      % apply longitudinal
      SV(i).v     = best_v_next;
      SV(i).theta = pi/2;  % straight
  end

  % --- Advance lane-change FSM & integrate Y
  for i=1:nSV
      SV(i).y = SV(i).y + SV(i).v*dt;   % along +Y

      if SV(i).lc_active
          SV(i).lc_t = SV(i).lc_t + dt;
          s = smoothstep_quintic(min(SV(i).lc_t/SV(i).lc_T,1.0));
          x_from = lane_center_of(SV(i).lane_from, lane_center_main, lane_center_left, lane_center_right);
          x_to   = lane_center_of(SV(i).lane_to,   lane_center_main, lane_center_left, lane_center_right);
          SV(i).x = (1-s)*x_from + s*x_to;

          if SV(i).lc_t >= SV(i).lc_T
              SV(i).lc_active = false;
              SV(i).lane_cur  = SV(i).lane_to;
              SV(i).x         = x_to;   % snap exactly
          end
      else
          SV(i).x = lane_center_of(SV(i).lane_cur, lane_center_main, lane_center_left, lane_center_right);
      end

      % recenter lock handling
      if SV(i).recenter && (SV(i).lane_cur=="MAIN")
          SV(i).ok_ctr = SV(i).ok_ctr + 1;
          if SV(i).ok_ctr >= HOLD_TICKS
              SV(i).recenter  = false;
              SV(i).lock_main = false;
              SV(i).ok_ctr    = 0;
          end
      else
          SV(i).ok_ctr = 0;
      end

      % logs
      log(i).t(end+1,1)=t; log(i).x(end+1,1)=SV(i).x; log(i).y(end+1,1)=SV(i).y;
      log(i).v(end+1,1)=SV(i).v; log(i).theta(end+1,1)=SV(i).theta;
      risk_now = crpf(SV(i).x, SV(i).y, ov_x, ov_y, 0.0, 0,0, ...
                      m_O, size_O, kappa_O, CRPF_G, CRPF_ZETA, PSEUDO_EPS, PSEUDO_RHO);
      log(i).riskOV(end+1,1)=risk_now; log(i).mode(end+1,1)=SV(i).mode; log(i).lane(end+1,1)=SV(i).lane_cur;

      if SV(i).preced>0
          j = SV(i).preced; log(i).gap(end+1,1)=SV(j).y - SV(i).y; log(i).relv(end+1,1)=SV(j).v - SV(i).v;
      else
          log(i).gap(end+1,1)=NaN; log(i).relv(end+1,1)=NaN;
      end
  end

  % OV log
  logOV.t(end+1,1)=t; logOV.x(end+1,1)=ov_x; logOV.y(end+1,1)=ov_y; logOV.vy(end+1,1)=ov_vy;

  % instantaneous collision check (current poses)
  for i=1:nSV
      if hypot(SV(i).x-ov_x, SV(i).y-ov_y) <= Rsum, collision=true; break; end
      for j=i+1:nSV
          if hypot(SV(i).x-SV(j).x, SV(i).y-SV(j).y) <= 2*Rsum, collision=true; break; end
      end
  end
  
  if collision, break; end

  % ----- console table -----
  if t + eps >= next_print
      if ~printed_header
          fprintf('\n%-6s %-4s %-10s %9s %9s %7s %7s\n', ...
              't[s]','ID','MODE','x[m]','y[m]','lane','v[m/s]');
          fprintf('%s\n', repmat('-',1,64));
          printed_header = true;
      end

      for ii = 1:nSV
          fprintf('%-6.1f %-4s %-10s %9.2f %9.2f %7s %7.2f\n', ...
              t, sprintf('SV%d',ii), char(SV(ii).mode), ...
              SV(ii).x, SV(ii).y, char(SV(ii).lane_cur), SV(ii).v);
      end

      fprintf('%-6.1f %-4s %-10s %9.2f %9.2f %7s %7.2f\n\n', ...
          t, 'OV', '-', ov_x, ov_y, 'MAIN', hypot(ov_vx, ov_vy));

      next_print = t + PRINT_EVERY;
  end

end

%% --- Heatmap anchored at last leave-from-MAIN
first_leave_t = nan(nSV,1);
for i = 1:nSV
    li = string(log(i).lane);
    idx = find(li ~= "MAIN", 1, 'first');
    if ~isempty(idx), first_leave_t(i) = log(i).t(idx); end
end
anchor_t = max(first_leave_t, [], 'omitnan');
if isnan(anchor_t)
    anchor_ovx = logOV.x(1); anchor_ovy = logOV.y(1);
else
    [~,kA]   = min(abs(logOV.t - anchor_t));
    anchor_ovx = logOV.x(kA); anchor_ovy = logOV.y(kA);
end

allY = [logOV.y; vertcat(log(:).y)];
ymin = floor(min(allY)/10)*10 - 20;
ymax = ceil(max(allY)/10)*10 + 20;

gx2 = linspace(-10, +10, 241);
gy2 = linspace(ymin, ymax, 301);
[GX2,GY2] = meshgrid(gx2, gy2);
CR2 = arrayfun(@(x,y) crpf(x,y, anchor_ovx, anchor_ovy, ...
    0.0, 0,0, m_O, size_O, kappa_O, ...
    CRPF_G, CRPF_ZETA, PSEUDO_EPS, PSEUDO_RHO), GX2, GY2);

figure('Color','w'); hold on;
imagesc(gx2, gy2, log10(CR2+1e-9)); axis xy; colorbar;
title('CRPF Heatmap @ OV pose when last SV starts lane change (stationary OV, RIGHT lane only)'); xlabel('X [m]'); ylabel('Y [m]');
[~, hC] = contour(gx2, gy2, log10(CR2+1e-9), 10, 'LineWidth',0.6); set(hC,'LineColor',[0.9 0.9 0.9]);
xline(+10,'k:','LineWidth',2.0); xline(-10,'k:','LineWidth',2.0);
plot(TRACK_MAIN(:,1),  TRACK_MAIN(:,2),  'k--','LineWidth',0.8);
plot(TRACK_LEFT(:,1),  TRACK_LEFT(:,2),  'k--','LineWidth',0.8);
plot(TRACK_RIGHT(:,1), TRACK_RIGHT(:,2), 'k--','LineWidth',0.8);
plot(logOV.x, logOV.y, 'k--', 'LineWidth', 2);
for i=1:nSV
   plot(log(i).x, log(i).y, SV(i).style, 'Color', SV(i).color, 'LineWidth',2);
   scatter(log(i).x(1), log(i).y(1), 36, SV(i).color, 'filled', 'MarkerEdgeColor','k');
end
% crosses where a behind-SV leaves MAIN
crossX = []; crossY = [];
for i = 1:nSV
    li = string(log(i).lane); ti = log(i).t; yi = log(i).y;
    idx = find(li(2:end) ~= "MAIN" & li(1:end-1) == "MAIN") + 1;
    for k = idx.'
        ovxk = interp1(logOV.t, logOV.x, ti(k));
        ovyk = interp1(logOV.t, logOV.y, ti(k));
        if yi(k) < ovyk, crossX(end+1,1)=ovxk; crossY(end+1,1)=ovyk; end %#ok<AGROW>
    end
end
if ~isempty(crossX)
    uv = unique([crossX crossY], 'rows', 'stable');
    scatter(uv(:,1), uv(:,2), 70, 'kx', 'LineWidth', 2);
end
scatter(logOV.x(1), logOV.y(1), 60, 'ko', 'filled', 'MarkerEdgeColor','w');
xlim([-10 10]); ylim([ymin ymax]); grid on;

% Time plots
figure('Color','w');
tvec = log(1).t;
subplot(3,1,1); hold on; title('Speed'); for i=1:nSV, plot(tvec,log(i).v,'Color',SV(i).color,'LineWidth',1.3); end
yline(v_des_platoon,'k--','v_{des}'); ylabel('m/s'); grid on;
legend({'SV1','SV2','SV3'},'Location','best');
subplot(3,1,2); hold on; title('Headway to predecessor'); for i=2:nSV, plot(tvec,log(i).gap,'Color',SV(i).color,'LineWidth',1.3); end
yline(d_headway_des,'k--','d_{des}'); ylabel('m'); grid on;
subplot(3,1,3); hold on; title('OV CRPF @ SV pose'); for i=1:nSV, plot(tvec,log(i).riskOV,'Color',SV(i).color,'LineWidth',1.3); end
yline(CR_YELLOW_LOW,'k--'); yline(CR_YELLOW_HIGH,'k--'); ylabel('CRPF'); xlabel('t [s]'); grid on;

% Longitudinal Y vs time
figure('Color','w'); hold on; grid on;
for i=1:nSV, plot(log(i).t, log(i).y, 'LineWidth', 1.6, 'Color', SV(i).color); end
plot(logOV.t, logOV.y, 'k--', 'LineWidth', 1.8);
title('Longitudinal Position vs Time'); xlabel('Time [s]'); ylabel('Y [m]');
legend({'SV1','SV2','SV3','OV (stationary)'}, 'Location','best');

if collision
   fprintf('Collision flagged. Review trajectories.\n');
else
   fprintf('No collision detected.\n');
end

%% ===== helpers =====
function start_lane_change(i, next_lane)
    SV(i).lane_from = SV(i).lane_cur;
    SV(i).lane_to   = next_lane;
    SV(i).lc_t      = 0;
    SV(i).lc_active = true;
    SV(i).lane_target = next_lane;
    SV(i).track = lane_track_of(next_lane, TRACK_MAIN, TRACK_LEFT, TRACK_RIGHT);
end

function xc = lane_center_of(lbl, xM, xL, xR)
    if lbl=="MAIN", xc = xM; elseif lbl=="LEFT", xc = xL; else, xc = xR; end
end

function TR = lane_track_of(lbl, TM, TL, TRt)
    if lbl=="MAIN", TR = TM; elseif lbl=="LEFT", TR = TL; else, TR = TRt; end
end

function s = smoothstep_quintic(u)
    u = max(0,min(1,u)); s = 10*u^3 - 15*u^4 + 6*u^5;
end

function v = ternary(cond, a, b), if cond, v=a; else, v=b; end, end

function free = lane_window_free(xc, y0, y1, ovx, ovy, SV, i)
    free = ~((abs(ovx - xc) < 1.5) && (ovy>=y0 && ovy<=y1));
    for jj=1:numel(SV)
        if jj==i, continue; end
        if (abs(SV(jj).x - xc) < 1.5) && (SV(jj).y>=y0 && SV(jj).y<=y1)
            free = false; return;
        end
    end
end

function alpha = gate_alpha(risk_now, rlow, rhigh)
    if risk_now <= rlow, alpha = 0;
    elseif risk_now >= rhigh, alpha = 1;
    else, tau = (risk_now - rlow)/max(1e-12,(rhigh-rlow)); alpha = tau.^2.*(3-2*tau); end
end

% ---------- Section III-B helper functions ----------
function val = crpf(xs, ys, xo, yo, v_obs, ax, ay, m_obs, size_obs, kappa_obs, G, zeta, eps, rho)
    rd = pseudo_distance(xs, ys, xo, yo, v_obs, eps, rho);
    rd = max(rd, 2.5);
    M  = virtual_mass(m_obs, v_obs);
    Tfac = type_factor(size_obs, kappa_obs);
    phi  = accel_factor(ax, ay, xs, ys, xo, yo);
    val = G * M * Tfac * phi / (rd^zeta);
end
function rd = pseudo_distance(xs, ys, xo, yo, v_obs, eps, rho)
    dx = xs - xo; dy = ys - yo;
    termx = eps * dx * exp(-rho * max(v_obs,0.0));
    termy = eps * dy;
    rd = hypot(termx, termy);
end
function phi = accel_factor(ax, ay, xs, ys, xo, yo)
    K = 5.0; a=[ax,ay]; rd=[xs-xo, ys-yo]; na=norm(a); nr=norm(rd);
    if na<1e-9 || nr<1e-9, phi=1.0; return; end
    cos_t = dot(a/na, rd/nr); denom = K - na*cos_t; denom = min(max(denom,0.2),10.0); phi = K/denom;
end
function M = virtual_mass(m, v), M = m * (1.566e-14 * (max(v,0.0)^6.687) + 0.3354); end
function Tfac = type_factor(size, kappa)
    size_star = 4.0; kappa_star = 1.0; w1=1.0; w2=1.0;
    Tfac = (size / max(size_star,1e-6))^w1 * (kappa / max(kappa_star,1e-6))^w2;
end
function [collide, ttc] = will_collide(p_rel, v_rel, R, horizon)
    v2 = dot(v_rel, v_rel);
    if v2 < 1e-12, collide=(norm(p_rel)<=R); ttc=inf; return; end
    tstar = -dot(p_rel, v_rel)/v2; tstar = min(max(tstar,0.0), horizon);
    dmin = norm(p_rel + tstar*v_rel); ttc = inf;
    if dot(p_rel, v_rel) < 0
        a=v2; b=2*dot(p_rel, v_rel); c=dot(p_rel,p_rel)-R^2; disc = b*b - 4*a*c;
        if disc>=0, r1=(-b - sqrt(disc))/(2*a); r2=(-b + sqrt(disc))/(2*a);
            roots=[r1,r2]; roots=roots(roots>=0); if ~isempty(roots), ttc=min(roots); end
        end
    end
    collide = (dmin <= R);
end

% ---------- Track helpers ----------
function [cte, psi_err] = track_error(x, y, theta, TRACK)
    if size(TRACK,1)>=2 && all(abs(diff(TRACK(:,1)))<1e-9)
        x_ref = TRACK(1,1); cte = x - x_ref; psi_ref = pi/2; psi_err = wrapToPi(theta - psi_ref); return
    end
    [xp, yp, tx, ty] = nearest_point_on_polyline(x, y, TRACK);
    nx = -ty; ny = tx; cte = (x - xp)*nx + (y - yp)*ny;
    psi_ref = atan2(ty, tx); psi_err = wrapToPi(theta - psi_ref);
end
function [xp, yp, tx, ty] = nearest_point_on_polyline(x, y, P)
    best_d2=inf; xp=P(1,1); yp=P(1,2); tx=0; ty=1;
    for i=1:size(P,1)-1
        x0=P(i,1); y0=P(i,2); x1=P(i+1,1); y1=P(i+1,2);
        vx=x1-x0; vy=y1-y0; L2=vx*vx+vy*vy+1e-12;
        t=((x-x0)*vx + (y-y0)*vy)/L2; t=min(max(t,0.0),1.0);
        xc=x0 + t*vx; yc=y0 + t*vy;
        d2=(x-xc)^2 + (y-yc)^2;
        if d2<best_d2, best_d2=d2; xp=xc; yp=yc; L=sqrt(L2); tx=vx/L; ty=vy/L; end
    end
end
end