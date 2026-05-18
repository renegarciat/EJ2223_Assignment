function [left_magnet_points, right_magnet_points] = draw_rotor_sector(model, geom_tag, ...
                           D_r, D_ir, air_gap, p, ...
                           b_m, h_m, w_ib, h_ry, angle_m, ...
                           draw_only_sector)

% DRAW_ROTOR_SECTOR  Adds the rotor sector geometry to an existing COMSOL model.
%
% This function draws one pole sector of an IPM rotor cross-section into a
% COMSOL geometry node that was previously created by the stator script.
% It relies on the periodicity of the machine: only 1/(2p) of the full
% cross-section is modelled (one pole sector = 360 deg / (2p)) entirely in
% the upper half-plane. The magnets are centred on the iron core.
%
% The function calls geom.run() internally to finalise the geometry and
% then creates named selections via indexes (those indexes can be obtained
% in the COMSOL GUI)
%
% Do NOT call geom.run() again from the stator script after this function.
%
% -------------------------------------------------------------------------
% INPUTS
% -------------------------------------------------------------------------
%   model       - COMSOL model object
%   geom_tag    - char, tag of the geometry node (e.g. 'geom1')
%
%   D_r         - [m]   Rotor outer radius (= stator inner radius - air gap)
%   D_ir        - [m]   Inner radius of rotor lamination
%   air_gap     - [m]   Air gap radius
%   p           - [-]   Number of pole pairs
%
%   b_m         - [m]   Magnet length
%   h_m         - [m]   Magnet width
%   w_ib        - [m]   Magnet spacing
%   h_ry        - [m]   Magnet spacing with inner radius
%   angle_m     - [rad] Magnet angle
%
%   draw_only_sector    - [bool]    Specify if only one sector of the motor
%                                   has to be drawn
% -------------------------------------------------------------------------
% SELECTIONS CREATED - usefull for the material part
% -------------------------------------------------------------------------
%   'sel_iron'                  - rotor and stator lamination domain
%   'sel_shaft'                 - shaft domain
%   'sel_airgap'                - air-gap domain
%   'sel_rotor_magnets'         - all the magnet domains
%   'sel_rotor_left_magnets'    - all the magnet domains with a high sector-to-magnet angle
%   'sel_rotor_right_magnets'   - all the magnet domains with a low sector-to-magnet angle
%   'bnd_inner_magnet_right     - inner boundaries of the right magnets
%   'bnd_outer_magnet_right     - outer boundaries of the right magnets
%   'bnd_inner_magnet_left      - inner boundaries of the left magnets
%   'bnd_outer_magnet_left      - outer boundaries of the left magnets
%
%   If draw_only_sector is true,
%       'bnd_sector_sides'      - boundary of the sector sides
%
% =========================================================================

    fprintf('[draw_rotor_sector] Creating rotor geometry...\n');

    %% ---- 1. Sector geometry parameters --------------------------------
    n_poles      = 2 * p;
    sector_angle = 2*pi / n_poles;      % one pole sector [rad]
    half_sector  = sector_angle / 2;    % d-axis angle (bisector of sector)

    fprintf('  Number of poles pairs            : %d\n', p);
    fprintf('  Only one sector drawn            : %s\n', mat2str(draw_only_sector));

    %% ---- 2. Retrieve the geometry node --------------------------------
    geom = model.geom(geom_tag);

    %% ---- 3. Rotor sector outline (pie slice) --------------------------
    
    function draw_centered_circle(radius, name)
        circle = geom.feature.create(name, 'Circle');
        circle.set('r',   radius);
        circle.set('pos', [0, 0]);
    end

    draw_centered_circle(D_r + air_gap, 'c_air_gap')
    fprintf('  Air-gap length                   : %.2f mm\n', air_gap*1e3);
    draw_centered_circle(D_r,           'c_rotor')
    fprintf('  Rotor outer radius               : %.2f mm\n', D_r*1e3);
    draw_centered_circle(D_ir,          'c_shaft')
    fprintf('  Rotor inner radius               : %.2f mm\n', D_ir*1e3);

    function crop_trace_to_sector(sector_angle, trace_name)
        % Sector mask — pie-slice triangle from angle_start to angle_end
        r_mask = (D_r + air_gap) * 2;   % Very long mask to be sure that it includes the whole rotor
        poly_x = [0, r_mask,    r_mask * cos(sector_angle)];
        poly_y = [0, 0,         r_mask * sin(sector_angle)];
    
        % Clip rotor disk to sector
        mask = geom.feature.create(join(['sector_mask_', trace_name]), 'Polygon');
        mask.set('x', poly_x);
        mask.set('y', poly_y);
        int = geom.feature.create(join(['int_sector_', trace_name]), 'Intersection');
        int.selection('input').set({trace_name, join(['sector_mask_', trace_name])});
    end

    if draw_only_sector
        crop_trace_to_sector(sector_angle, 'c_air_gap')
        crop_trace_to_sector(sector_angle, 'c_rotor')
        crop_trace_to_sector(sector_angle, 'c_shaft')
    end

    %% ---- 4. Magnet pocket — polygons ----------------
    % Each pocket is defined as a trapezoid (4 corners):
    % using polar coordinates then converted to Cartesian.
    
    function points = make_magnet(center_angle, b_m, h_m, w_ib_2, h_ry, angle_m, name)
        c = w_ib_2 - h_m*cos(angle_m);
        r = sqrt((D_ir + h_ry)^2 + c^2);
        t = atan(c/(D_ir+h_ry));
        d = center_angle+pi/2 - angle_m;

        % Helper: polar -> Cartesian columns [x; y]
        pol2cart_pts = @(r, th) [r .* cos(th); r .* sin(th)];

        point1 = pol2cart_pts(r, center_angle + t);
        point2 = point1 + pol2cart_pts(h_m, d);
        point3 = point2 + pol2cart_pts(b_m, d-pi/2);
        point4 = point3 + pol2cart_pts(h_m, -(pi-d));
        
        points = [point1, point2, point3, point4];

        polygon = geom.feature.create(name, 'Polygon');
        polygon.set('x', points(1,:));
        polygon.set('y', points(2,:));
    end
        
    n_sector_to_draw = n_poles - (n_poles - 1) * draw_only_sector;
    
    right_magnet_points = zeros(2, 4, n_sector_to_draw);
    left_magnet_points  = zeros(2, 4, n_sector_to_draw);
    
    for i = 0:n_sector_to_draw-1
        right_magnet_points(:, :, i+1) = make_magnet(half_sector + sector_angle * i, b_m,  h_m, -w_ib/2, h_ry,  angle_m, join(['rect_magnet_right_sector_', num2str(i+1)]));
        left_magnet_points(:, :, i+1)  = make_magnet(half_sector + sector_angle * i, b_m, -h_m,  w_ib/2, h_ry, -angle_m, join(['rect_magnet_left_sector_',  num2str(i+1)]));
    end

    fprintf('  Magnet length                    : %.2f mm\n', b_m*1e3);
    fprintf('  Magnet width                     : %.2f mm\n', h_m*1e3);
    fprintf('  Magnet spacing                   : %.2f mm\n', w_ib*1e3);
    fprintf('  Magnet spacing with inner radius : %.2f mm\n', h_ry*1e3);
    fprintf('  Magnet angle                     : %.2f °\n', angle_m/(2*pi)*360);

    %% ---- 5. Finalise geometry -----------------------------------------
    geom.run();

    fprintf('[draw_rotor_sector] Geometry completed successfully.\n');
end