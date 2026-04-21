function demo_crpf_III_B()
% Reproduce Fig. 3 for Section III-B
p = crpf_params();

% Grid (meters)
x = -100:1:100;  y = -100:1:100;
[X, Y] = meshgrid(x, y);

cases = { struct('v',0,  'a',[0,0],  'title','(a) v=0, a=0'), ...
          struct('v',10, 'a',[0,0],  'title','(b) v=10, a=0'), ...
          struct('v',10, 'a',[2,0],  'title','(c) v=10, a=2'), ...
          struct('v',10, 'a',[-2,0], 'title','(d) v=10, a=-2') };

% Precompute and pick a visualization cap that ignores the very top spike
Emags = cell(1, numel(cases));
for k = 1:numel(cases)
    [Emags{k}, ~, ~] = crpf_field(X, Y, cases{k}.v, cases{k}.a, p);
end
% Use 99.5th percentile to avoid a single-pixel peak flattening everything
vis_cap = max( cellfun(@(E) prctile(E(:), 99.5), Emags) );

tiledlayout(2,4,'TileSpacing','compact','Padding','compact');
for k = 1:numel(cases)
    E = Emags{k};

    % 3D surface
    nexttile(k);
    s = surf(X, Y, E, 'EdgeColor','none'); view([-35 35]); axis tight
    colormap turbo; caxis([0 vis_cap]);
    title(cases{k}.title,'FontWeight','bold');
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('|E|');
    cb = colorbar; cb.Label.String = '|E|';

    % Filled contours
    nexttile(k+4);
    contourf(X, Y, E, 40, 'LineStyle','none');
    axis equal tight; caxis([0 vis_cap]); colormap turbo;
    xlabel('X [m]'); ylabel('Y [m]');
    title('equipotential map');
end
set(gcf,'Color','w');
end
