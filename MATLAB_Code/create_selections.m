function create_selections(model, geom_tag, draw_only_sector, rotor_angle, ...
    left_magnet_points, right_magnet_points, tip_pocket_points, slot_points, ...
    D_ir, D_r, D_si, D_so, p)
    
% -------------------------------------------------------------------------
% SELECTIONS CREATED - usefull for the material part
% -------------------------------------------------------------------------
%   'sel_iron'                  - rotor and stator lamination domain
%   'sel_shaft'                 - shaft domain
%   'sel_airgap'                - air-gap domain
%   'sel_rotor_magnets'         - all the magnet domains
%   'sel_rotor_left_magnets'    - all the magnet domains with a high sector-to-magnet angle
%   'sel_rotor_right_magnets'   - all the magnet domains with a low sector-to-magnet angle
%   'bnd_south_magnet_right     - south boundaries of the right magnets
%   'bnd_north_magnet_right     - north boundaries of the right magnets
%   'bnd_south_magnet_left      - south boundaries of the left magnets
%   'bnd_north_magnet_left      - north boundaries of the left magnets
%
%   If draw_only_sector is true,
%       'bnd_sector_sides'      - boundary of the sector sides

    n_poles      = 2 * p;
    sector_angle = 2*pi / n_poles;      % one pole sector [rad]
    n_sector_to_draw = n_poles - (n_poles - 1) * draw_only_sector;

    %% ---- 1. Named selections (domains) --------------------------------

    function create_sel(tag, label, dim, ids)
        sel = model.selection.create(tag, 'Explicit');
        sel.label(label);
        sel.geom(geom_tag, dim);
        sel.set(ids);
    end

    function selected_ids = make_sel_by_point(tag, label, dim, points, excluded_ids)
        if dim == 2
            type = 'domain';
        else
            type = 'boundary';
        end

        indices = cell(1, size(points, 2));
        for k = 1:size(points, 2)
            indices{k} = mphselectcoords(model, geom_tag, points(:,k)', type, 'radius', 1e-4);
        end
        indices = unique([indices{:}]);
        selected_ids = setdiff(indices, excluded_ids);

        create_sel(tag, label, dim, selected_ids)
    end

    rotor_angle = rotor_angle * draw_only_sector;
    
    domains_adj_inner = mphselectcoords(model, geom_tag, [D_ir*cos(rotor_angle), D_ir*sin(rotor_angle)], 'domain');
    domains_adj_outer = mphselectcoords(model, geom_tag, [D_r*cos(rotor_angle), D_r*sin(rotor_angle)], 'domain');
    id_rotor_iron = intersect(domains_adj_inner, domains_adj_outer);
    id_stator_iron = mphselectcoords(model, geom_tag, [D_so, 0], 'domain');
    create_sel('sel_iron', 'Iron laminations', 2, [id_rotor_iron, id_stator_iron])

    id_shaft = make_sel_by_point('sel_shaft', 'Shaft', 2, [D_ir*cos(rotor_angle); D_ir*sin(rotor_angle)], id_rotor_iron);
    
    id_airgap = make_sel_by_point('sel_airgap_pockets', 'Air gap and pockets', 2, [[D_r*cos(rotor_angle); D_r*sin(rotor_angle)], tip_pocket_points], id_rotor_iron);
    
    pocket_ids = make_sel_by_point('sel_pockets', 'Air pockets', 2, tip_pocket_points, id_rotor_iron);
    magnets_ids = make_sel_by_point('sel_rotor_magnets', 'Rotor magnets', 2, [squeeze(left_magnet_points(:,1,:)), squeeze(right_magnet_points(:,1,:))], id_rotor_iron);
    create_sel('sel_rotor', 'Rotor', 2, [id_rotor_iron, id_shaft, pocket_ids, magnets_ids])
    
    make_sel_by_point('sel_rotor_left_magnets', 'Rotor left magnets', 2, squeeze(left_magnet_points(:,1,:)), id_rotor_iron);
    make_sel_by_point('sel_rotor_right_magnets', 'Rotor right magnets', 2, squeeze(right_magnet_points(:,1,:)), id_rotor_iron);

    create_sel('sel_all_without_copper', 'All the selections without the copper', 2, [id_rotor_iron, id_stator_iron, id_shaft, magnets_ids, id_airgap])

    n_points = size(slot_points,2);
    make_sel_by_point('sel_coils', 'Coils', 2, squeeze(slot_points(:,n_points/2,:)), id_stator_iron);
    make_sel_by_point('sel_coils_a_plus', 'Coil a+', 2, squeeze(slot_points(:,n_points/2,1:6:end)), id_stator_iron);
    make_sel_by_point('sel_coils_c_minus', 'Coil c-', 2, squeeze(slot_points(:,n_points/2,2:6:end)), id_stator_iron);
    make_sel_by_point('sel_coils_b_plus', 'Coil b+', 2, squeeze(slot_points(:,n_points/2,3:6:end)), id_stator_iron);
    make_sel_by_point('sel_coils_a_minus', 'Coil a-', 2, squeeze(slot_points(:,n_points/2,4:6:end)), id_stator_iron);
    make_sel_by_point('sel_coils_c_plus', 'Coil c+', 2, squeeze(slot_points(:,n_points/2,5:6:end)), id_stator_iron);
    make_sel_by_point('sel_coils_b_minus', 'Coil b-', 2, squeeze(slot_points(:,n_points/2,6:6:end)), id_stator_iron);

    %% ---- 2. Named selections (boundaries) -----------------------------

    function id_boundary = find_boundary_between_points(pointA, pointB)
        boundariesA = mphselectcoords(model, geom_tag, pointA, 'boundary');
        boundariesB = mphselectcoords(model, geom_tag, pointB, 'boundary');
        id_boundary = intersect(boundariesA, boundariesB);
    end

    magnets_bnd = zeros(n_sector_to_draw, 4);
    for i = 1:n_sector_to_draw
        invert_magnets = mod(i,2);
        right_points = right_magnet_points(:, :, i);
        magnets_bnd(i, 1+invert_magnets) = find_boundary_between_points(right_points(:,2), right_points(:,3));
        magnets_bnd(i, 2-invert_magnets) = find_boundary_between_points(right_points(:,4), right_points(:,1));
        
        left_points = left_magnet_points(:, :, i);
        magnets_bnd(i, 3+invert_magnets) = find_boundary_between_points(left_points(:,2), left_points(:,3));
        magnets_bnd(i, 4-invert_magnets) = find_boundary_between_points(left_points(:,4), left_points(:,1));
    end

    create_sel('bnd_south_magnet_right', 'South boundaries of right magnets', 1, squeeze(magnets_bnd(:,1)))
    create_sel('bnd_north_magnet_right', 'North boundaries of right magnets', 1, squeeze(magnets_bnd(:,2)))
    create_sel('bnd_south_magnet_left', 'South boundaries of Left magnets', 1, squeeze(magnets_bnd(:,3)))
    create_sel('bnd_north_magnet_left', 'North boundaries of Left magnets', 1, squeeze(magnets_bnd(:,4)))

    if draw_only_sector
        bnd_shaft_left = find_boundary_between_points([0, 0], [D_ir*cos(sector_angle + rotor_angle), D_ir*sin(sector_angle + rotor_angle)]);
        bnd_shaft_right = find_boundary_between_points([0, 0], [D_ir*cos(rotor_angle), D_ir*sin(rotor_angle)]);

        bnd_iron_stator_left = find_boundary_between_points([D_ir*cos(sector_angle + rotor_angle), D_ir*sin(sector_angle + rotor_angle)], [D_r*cos(sector_angle + rotor_angle), D_r*sin(sector_angle + rotor_angle)]);
        bnd_iron_stator_right = find_boundary_between_points([D_ir*cos(rotor_angle), D_ir*sin(rotor_angle)], [D_r*cos(rotor_angle), D_r*sin(rotor_angle)]);

        create_sel('bnd_rotor_sides', 'Outer boundaries of the rotor', 1, [bnd_shaft_left, bnd_shaft_right, bnd_iron_stator_left, bnd_iron_stator_right])
        
        bnd_airgap_left = find_boundary_between_points([D_r*cos(sector_angle), D_r*sin(sector_angle)], [D_si*cos(sector_angle), D_si*sin(sector_angle)]);
        bnd_airgap_right = find_boundary_between_points([D_r, 0], [D_si, 0]);
        
        bnd_iron_rotor_left = find_boundary_between_points([D_si*cos(sector_angle), D_si*sin(sector_angle)], [D_so*cos(sector_angle), D_so*sin(sector_angle)]);
        bnd_iron_rotor_right = find_boundary_between_points([D_si, 0], [D_so, 0]);

        create_sel('bnd_stator_sides', 'Outer boundaries of the stator', 1, [bnd_airgap_left, bnd_airgap_right, bnd_iron_rotor_left, bnd_iron_rotor_right])
    end
end