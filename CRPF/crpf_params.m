function p = crpf_params()
% Parameters for CRPF (Section III-B)

% Fig. 3 caption values
p.G    = 0.5;   % Eq. (11)
p.R    = 1.0;   % Eq. (10) -> set to 1 for Fig. 3
p.K    = 5.0;   % Eq. (9)
p.T    = 1.0;   % Eq. (7)
p.zeta = 1.2;   % Eq. (11)

% Undetermined coefficients from Eq. (8) (paper leaves ε, ρ open)
p.eps_aniso = 1.0;    % ε
p.rho       = 0.02;   % ρ  (makes (b–d) slightly elliptical as in Fig. 3)

% Virtual mass – absolute scale (m) only changes overall magnitude
p.m = 1.0;

% **Important**: avoid singularity at the obstacle center.
% Use a realistic “body radius” (meters) so the top doesn’t dominate the colorbar.
p.rd_eps = 5.0;       % ~vehicle size scale; try 3–8 if you want a sharper/flatter peak
end
