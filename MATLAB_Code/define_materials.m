function define_materials(model, comp_tag, phys_tag, draw_only_sector, mesh_size, ...
                            mu_r_shaft, sigma_shaft, epsilon_r_shaft, ...
                            mu_r_iron, sigma_iron, epsilon_r_iron, ...
                            mu_r_air, sigma_air, ...
                            mu_r_magnets, sigma_magnets, Br)
    fprintf('[define_materials] Defining materials...\n');

    %% ---- 1. Retrieving component ---------------------------------------

    comp = model.component(comp_tag);

    %% ---- 2. All materials ----------------------------------------------
    function mat = def_mat(tag, name, mu, sigma, selection)
        mat = comp.material.create(tag);
        mat.label(name);
        mat.materialmodel('def').set('relpermeability', mu);
        mat.materialmodel('def').set('electricconductivity', sigma);
        mat.selection.named(selection);
    end

    % Shaft
    mat_shaft = def_mat('mat_shaft', 'Shaft', mu_r_shaft, sigma_shaft, 'sel_shaft');
    mat_shaft.materialmodel('def').set('relpermittivity', epsilon_r_shaft);    
    fprintf('  Shaft relative permeability     : %.2f\n', mu_r_shaft);
    fprintf('  Shaft conductivity              : %.2f S/m\n', sigma_shaft);
    fprintf('  Shaft relative permittivity     : %.2f\n', epsilon_r_shaft);

    % Iron
    mat_iron = def_mat('mat_iron', 'Iron', mu_r_iron, sigma_iron, 'sel_iron');
    mat_iron.materialmodel('def').set('relpermittivity', epsilon_r_iron);    
    fprintf('  Iron relative permeability      : %.2f\n', mu_r_iron);
    fprintf('  Iron conductivity               : %.2f S/m\n', sigma_iron);
    fprintf('  Iron relative permittivity      : %.2f\n', epsilon_r_iron);

    % Air
    def_mat('mat_air', 'Air', mu_r_air, sigma_air, 'sel_airgap');
    fprintf('  Air relative permeability       : %.2f\n', mu_r_air);
    fprintf('  Air conductivity                : %.2f S/m\n', sigma_air);

    % Magnets
    mat_mag = def_mat('mat_magnets', 'Magnets', mu_r_magnets, sigma_magnets, 'sel_rotor_magnets');
    mat_mag.propertyGroup.create('RemanentFluxDensity', 'RemanentFluxDensity', 'Remanent_flux_density');
    mat_mag.propertyGroup('RemanentFluxDensity').set('normBr', {num2str(Br)});
    fprintf('  Magnets relative permeability   : %.2f\n', mu_r_magnets);
    fprintf('  Magnets conductivity            : %.2f S/m\n', sigma_magnets);
    fprintf('  Magnets remanent flux density   : %.2f T\n', Br);

    %% ---- 3. Magnets physic ---------------------------------------------

    phys = comp.physics(phys_tag);

    function define_magnets(tag, name, selection, north_bnd, south_bnd)
        magnet_feature = phys.feature.create(tag, 'Magnet');
        magnet_feature.label(name);
        magnet_feature.selection.named(selection);
        magnet_feature.set('DirectionMethod', 'SpecifyNorthSouthBoudaries');

        north_feature = magnet_feature.feature('north1');
        north_feature.selection().named(north_bnd);
    
        south_feature = magnet_feature.feature('south1');
        south_feature.selection().named(south_bnd);
    end
    
    define_magnets('feature_right_magnets', 'Right magnets physic', 'sel_rotor_right_magnets', 'bnd_inner_magnet_right', 'bnd_outer_magnet_right')
    define_magnets('feature_left_magnets', 'Left magnets physic', 'sel_rotor_left_magnets', 'bnd_outer_magnet_left', 'bnd_inner_magnet_left')

    %% ---- 4. Periodic conditions ---------------------------------------

    if draw_only_sector
        per = phys.feature.create('periodic_conditions', 'PeriodicCondition', 1);
        per.label('Periodic conditions')
        per.selection.named('bnd_sector_sides');
        per.set('PeriodicType', 'AntiPeriodicity');
    end
    fprintf('[define_materials] All materials assigned successfully.\n');

    amp = phys.feature.create('ampere_law_condition', 'AmperesLawSolid', 2);
    amp.label("Ampère's law");
    amp.selection.named('sel_iron_shaft');

    %% ---- 5. Mesh -------------------------------------------------------
    
    comp_tag = 'mesh1';
    mesh_obj = comp.mesh.create(comp_tag);
    mesh_obj.label('Mesh');
    mesh_obj.autoMeshSize(mesh_size);
    mesh_obj.run();
end