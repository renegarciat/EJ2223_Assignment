function create_selections(model, geom_tag, draw_only_sector, left_magnet_points, right_magnet_points, ...
    D_ir, D_r, D_si, D_so, p)
    
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

    domains_adj_inner = mphselectcoords(model, geom_tag, [D_ir, 0], 'domain');
    domains_adj_outer = mphselectcoords(model, geom_tag, [D_r, 0], 'domain');
    id_rotor_iron = intersect(domains_adj_inner, domains_adj_outer);
    id_stator_iron = mphselectcoords(model, geom_tag, [D_so, 0], 'domain');
    create_sel('sel_iron', 'Iron laminations', 2, [id_rotor_iron, id_stator_iron])

    id_shaft = make_sel_by_point('sel_shaft', 'Shaft', 2, [D_ir; 0], id_rotor_iron);
    create_sel('sel_iron_shaft', 'Iron laminations and shaft', 2, [id_rotor_iron, id_stator_iron, id_shaft])
    
    make_sel_by_point('sel_airgap', 'Air gap', 2, [D_r; 0], id_rotor_iron);
    make_sel_by_point('sel_rotor_magnets', 'Rotor magnets', 2, [squeeze(left_magnet_points(:,1,:)), squeeze(right_magnet_points(:,1,:))], id_rotor_iron);
    make_sel_by_point('sel_rotor_left_magnets', 'Rotor left magnets', 2, squeeze(left_magnet_points(:,1,:)), id_rotor_iron);
    make_sel_by_point('sel_rotor_right_magnets', 'Rotor right magnets', 2, squeeze(right_magnet_points(:,1,:)), id_rotor_iron);

    %% ---- 2. Named selections (boundaries) -----------------------------

    function id_boundary = find_boundary_between_points(pointA, pointB)
        boundariesA = mphselectcoords(model, geom_tag, pointA, 'boundary');
        boundariesB = mphselectcoords(model, geom_tag, pointB, 'boundary');
        id_boundary = intersect(boundariesA, boundariesB);
    end

    magnets_bnd = zeros(n_sector_to_draw, 4);
    for i = 1:n_sector_to_draw
        right_points = right_magnet_points(:, :, i);
        magnets_bnd(i, 1) = find_boundary_between_points(right_points(:,1), right_points(:,2));
        magnets_bnd(i, 2) = find_boundary_between_points(right_points(:,3), right_points(:,4));
        
        left_points = left_magnet_points(:, :, i);
        magnets_bnd(i, 3) = find_boundary_between_points(left_points(:,1), left_points(:,2));
        magnets_bnd(i, 4) = find_boundary_between_points(left_points(:,3), left_points(:,4));
    end

    create_sel('bnd_inner_magnet_right', 'Inner boundaries of right magnets', 1, squeeze(magnets_bnd(:,1)))
    create_sel('bnd_outer_magnet_right', 'Outer boundaries of right magnets', 1, squeeze(magnets_bnd(:,2)))
    create_sel('bnd_inner_magnet_left', 'Inner boundaries of Left magnets', 1, squeeze(magnets_bnd(:,3)))
    create_sel('bnd_outer_magnet_left', 'Outer boundaries of Left magnets', 1, squeeze(magnets_bnd(:,4)))

    if draw_only_sector
        bnd_left_shaft = mphselectcoords(model, geom_tag, [D_ir*cos(sector_angle), D_ir*sin(sector_angle)], 'boundary');
        bnd_right_shaft = mphselectcoords(model, geom_tag, [D_ir, 0], 'boundary');
        id_shaft_stator_bnd = setxor(bnd_left_shaft, bnd_right_shaft);

        bnd_left_iron = mphselectcoords(model, geom_tag, [D_r*cos(sector_angle), D_r*sin(sector_angle)], 'boundary');
        bnd_right_iron = mphselectcoords(model, geom_tag, [D_r, 0], 'boundary');
        id_stator_airgap_bnd = setxor(bnd_left_iron, bnd_right_iron);
        
        bnd_left_stator = find_boundary_between_points([D_si*cos(sector_angle), D_si*sin(sector_angle)], [D_so*cos(sector_angle), D_so*sin(sector_angle)])
        bnd_right_stator = find_boundary_between_points([D_si, 0], [D_so, 0])

        create_sel('bnd_sector_sides', 'Outer boundaries of the sector sides', 1, [id_shaft_stator_bnd, id_stator_airgap_bnd, bnd_left_stator, bnd_right_stator])
    end
end