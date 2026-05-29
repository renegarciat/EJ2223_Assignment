function slot_points = draw_stator_sector(model, geom_tag, ...
                            r_si, ...
                            r_so, ...
                            air_gap, ...
                            Qs, ...
                            d_os, ...
                            d_s, ...
                            b1, ...
                            b2, ...
                            r1, ...
                            p, ...
                            draw_only_sector)
    NUMBER_OF_POINTS_IN_ROUNDED_CORNER = 5;

%==========================================================================
% DRAW_STATOR_SECTOR
%
% Creates:
%   - stator geometry
%   - copper regions
%   - automatic sector/full model
%   - selections:
%       stator iron
%       copper up
%       copper down
%       left/right periodic boundaries
%   Arguments: 
%   - r_si: Stator Inner Radius
%==========================================================================

fprintf('[draw_stator_sector] Creating stator geometry...\n');

%% ------------------------------------------------------------------------
% Geometry object
% -------------------------------------------------------------------------

geom = model.component('comp1').geom(geom_tag);

%% ------------------------------------------------------------------------
% SLOT DRAWING
% -------------------------------------------------------------------------

% Helper: polar -> Cartesian columns [x; y]
pol2cart_pts = @(r, th) [r .* cos(th); r .* sin(th)];
function point = coord_offseted_points(alpha, l, offset)
    theta = atan(offset/l);
    D = sqrt(offset^2 + l^2);
    point = pol2cart_pts(D, alpha + theta);
end

function arc_pts = arc_chamfer(p1, p2, p_ref, r1, n_pts)
    % p_ref : reference point indicating which side the chamfer should be on

    mid = (p1 + p2) / 2;
    d = norm(p2 - p1);
    h = sqrt(r1^2 - (d/2)^2);
    perp = [-(p2(2)-p1(2)); p2(1)-p1(1)] / d;

    center_plus  = mid + h * perp;
    center_minus = mid - h * perp;

    % Choose the center closest to the reference point
    if norm(center_plus - p_ref) < norm(center_minus - p_ref)
        center = center_plus;
    else
        center = center_minus;
    end

    theta1 = atan2(p1(2) - center(2), p1(1) - center(1));
    theta2 = atan2(p2(2) - center(2), p2(1) - center(1));

    delta = mod((theta2 - theta1) + pi, 2*pi) - pi;
    theta2 = theta1 + delta;

    thetas = linspace(theta1, theta2, n_pts);
    arc_pts = center + r1 * [cos(thetas); sin(thetas)];
end

function draw_polygon(points, name)
    polygon = geom.feature.create(name, 'Polygon');
    polygon.set('x', points(1,:));
    polygon.set('y', points(2,:));
end

function slot_points = draw_slot_at_angle(alpha_slot, d_os, d_s, b1, b2, r1, name)
    l = r_si + d_os;

    point1 = coord_offseted_points(alpha_slot, l,           b1/2-r1);
    point2 = coord_offseted_points(alpha_slot, l+r1,        b1/2);
    arc1 = arc_chamfer(point1, point2, ...
        coord_offseted_points(alpha_slot, l+r1, b1/2-r1), r1, NUMBER_OF_POINTS_IN_ROUNDED_CORNER);
    
    point3 = coord_offseted_points(alpha_slot, l+d_s-r1,    b2/2);
    point4 = coord_offseted_points(alpha_slot, l+d_s,       b2/2-r1);
    arc2 = arc_chamfer(point3, point4, ...
        coord_offseted_points(alpha_slot, l+d_s-r1, b2/2-r1), r1, NUMBER_OF_POINTS_IN_ROUNDED_CORNER);
    
    point5 = coord_offseted_points(alpha_slot, l+d_s,       -(b2/2-r1));
    point6 = coord_offseted_points(alpha_slot, l+d_s-r1,    -b2/2);
    arc3 = arc_chamfer(point5, point6, ...
        coord_offseted_points(alpha_slot, l+d_s-r1, -(b2/2-r1)), r1, NUMBER_OF_POINTS_IN_ROUNDED_CORNER);
    
    point7 = coord_offseted_points(alpha_slot, l+r1,        -b1/2);
    point8 = coord_offseted_points(alpha_slot, l,           -(b1/2-r1));
    arc4 = arc_chamfer(point7, point8, ...
        coord_offseted_points(alpha_slot, l+r1, -(b1/2-r1)), r1, NUMBER_OF_POINTS_IN_ROUNDED_CORNER);

    pointA = coord_offseted_points(alpha_slot, r_si/2, b1/2-r1);
    pointD = coord_offseted_points(alpha_slot, r_si/2, -(b1/2-r1));
    
    slot_points = [arc1, arc2, arc3, arc4];

    draw_polygon(slot_points, join([name, '_copper']))
    draw_polygon([pointA, arc1, arc2, arc3, arc4, pointD], join(['dif_', name]))
    draw_polygon([pointA, arc1(:, 1), arc4(:, end), pointD], join([name, '_airpath']))        
end

slot_pitch = 2*pi/Qs;
i=0;
slot_names = cell(2, Qs);
slot_points = zeros(2, 4 * NUMBER_OF_POINTS_IN_ROUNDED_CORNER, Qs);

for alpha_slot = 0:slot_pitch:2*pi-slot_pitch
    i=i+1;
    slot_names{1,i} = sprintf('slot_%d_airpath', i);
    slot_names{2,i} = sprintf('dif_slot_%d', i);
    slot_points(:,:,i) = draw_slot_at_angle(alpha_slot, d_os, d_s, b1, b2, r1, join(['slot_', num2str(i)]));
end

%% ------------------------------------------------------------------------
% Create stator circles
% -------------------------------------------------------------------------
function draw_centered_circle(radius, name)
    circle = geom.feature.create(name, 'Circle');
    circle.set('r',   radius);
    circle.set('pos', [0, 0]);
end

draw_centered_circle(r_so, 'c_stator_outer')
draw_centered_circle(r_si, 'c_stator_inner')

dif_ring = geom.feature.create('tmp_stator', 'Difference');
dif_ring.selection('input').set('c_stator_outer');
dif_ring.selection('input2').set('c_stator_inner');

dif_ring = geom.feature.create('stator', 'Difference');
dif_ring.selection('input').set('tmp_stator');
dif_ring.selection('input2').set(slot_names(2,:));

%% ------------------------------------------------------------------------
% Create slots
% -------------------------------------------------------------------------
draw_centered_circle(r_si - air_gap/2, 'c_mid_airgap')
dif_ring = geom.feature.create('slots_air_path', 'Difference');
dif_ring.selection('input').set(slot_names(1,:));
dif_ring.selection('input2').set('c_mid_airgap');

%% ------------------------------------------------------------------------
% Create airgap
% -------------------------------------------------------------------------
draw_centered_circle(r_si, 'c_stator_inner_airgap')
draw_centered_circle(r_si-air_gap, 'c_airgap_inner')
dif_airgap = geom.feature.create('airgap', 'Difference');
dif_airgap.selection('input').set('c_stator_inner_airgap');
dif_airgap.selection('input2').set('c_airgap_inner');

union_slots_airgap = geom.feature.create('slots_airgap', 'Union');
union_slots_airgap.selection('input').set({'airgap' 'slots_air_path'});
union_slots_airgap.set('intbnd', false);

%% ------------------------------------------------------------------------
% Sector crop
% -------------------------------------------------------------------------

function crop_to_sector(sector_angle, r_mask, name_to_crop)
    poly_x = [0, r_mask,    r_mask * cos(sector_angle)];
    poly_y = [0, 0,         r_mask * sin(sector_angle)];

    % Clip stator disk to sector
    name = join(['sector_mask_', name_to_crop]);
    mask = geom.feature.create(name, 'Polygon');
    mask.set('x', poly_x);
    mask.set('y', poly_y);
    int = geom.feature.create(join(['int_sector_', name_to_crop]), 'Intersection');
    int.selection('input').set({name_to_crop, name});
end

if draw_only_sector
    n_poles = 2*p;
    sector_angle = 2*pi/n_poles;

    r_mask = 2*r_so;
    crop_to_sector(sector_angle, r_mask, 'stator')
    crop_to_sector(sector_angle, r_mask, 'slots_airgap')

    for i=1:1:Qs
        crop_to_sector(sector_angle, r_mask, join(['slot_', num2str(i), '_copper']))
    end
end

%% ------------------------------------------------------------------------
% Build geometry
% -------------------------------------------------------------------------

geom.run();

%% ------------------------------------------------------------------------
% Console feedback
% -------------------------------------------------------------------------

fprintf('[draw_stator_sector] Geometry completed.\n');

fprintf('  Sector model enabled : %s\n', ...
    mat2str(draw_only_sector));

fprintf('  Copper regions added successfully.\n');

end