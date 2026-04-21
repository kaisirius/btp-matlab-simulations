function [E_mag, E_x, E_y] = crpf_field(X, Y, v, a_vec, p)
% CRPF per Section III-B: Eq. (8) pseudo-distance, Eq. (9) accel indicator,
% Eq. (11) Yukawa-form EV.  (See paper lines for exact formulas.)
% Pseudo-distance (Eq. 8)
xprime = (X) .* p.eps_aniso .* exp(-p.rho * abs(v));
yprime = (Y) .* p.eps_aniso;
rd_mag = hypot(xprime, yprime);
rd_mag = max(rd_mag, p.rd_eps);  % <-- key change vs tiny values

ux = xprime ./ rd_mag;
uy = yprime ./ rd_mag;

% Acceleration indicator (Eq. 9)
a_norm = hypot(a_vec(1), a_vec(2));
if a_norm < 1e-12
    cos_th = zeros(size(X));
else
    ax_hat = a_vec(1) / a_norm;  ay_hat = a_vec(2) / a_norm;
    cos_th = ax_hat .* ux + ay_hat .* uy;
end
phi = p.K ./ (p.K - abs(a_norm) .* cos_th);

% Virtual mass (Eq. 6 inside Eq. 11)
Mfac = (1.566e-14 * v.^6.687 + 0.3354);
M    = p.m .* Mfac;

% CRPF magnitude and components (Eq. 11)
const = p.G .* M .* p.T .* p.R .* phi;
E_mag = const ./ (rd_mag .^ p.zeta);
E_x   = E_mag .* ux;
E_y   = E_mag .* uy;
end
