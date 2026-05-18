function draw_stator_sector(model, geom_tag, ...
                            r_si, ...
                            r_so, ...
                            Qs, ...
                            p, ...
                            slot_depth, ...
                            slot_width, ...
                            draw_only_sector)

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
%
%==========================================================================

fprintf('[draw_stator_sector] Creating stator geometry...\n');

%% ------------------------------------------------------------------------
% Geometry object
% -------------------------------------------------------------------------

geom = model.component('comp1').geom(geom_tag);

%% ------------------------------------------------------------------------
% Sector parameters
% -------------------------------------------------------------------------

n_poles = 2*p;

sector_angle = 2*pi/n_poles;

%% ------------------------------------------------------------------------
% Create stator circles
% -------------------------------------------------------------------------

c_outer = geom.feature.create('c_stator_outer', 'Circle');

c_outer.set('r', r_so);

c_outer.set('pos', [0 0]);

c_inner = geom.feature.create('c_stator_inner', 'Circle');

c_inner.set('r', r_si);

c_inner.set('pos', [0 0]);

%% ------------------------------------------------------------------------
% Sector crop
% -------------------------------------------------------------------------

if draw_only_sector

    r_mask = 2*r_so;

    poly_x = [ ...
        0 ...
        r_mask ...
        r_mask*cos(sector_angle)];

    poly_y = [ ...
        0 ...
        0 ...
        r_mask*sin(sector_angle)];

    mask = geom.feature.create('sector_mask', 'Polygon');

    mask.set('x', poly_x);

    mask.set('y', poly_y);

    int_outer = geom.feature.create( ...
        'int_outer', ...
        'Intersection');

    int_outer.selection('input').set( ...
        {'c_stator_outer' 'sector_mask'});

    int_inner = geom.feature.create( ...
        'int_inner', ...
        'Intersection');

    int_inner.selection('input').set( ...
        {'c_stator_inner' 'sector_mask'});

    outer_name = 'int_outer';

    inner_name = 'int_inner';

else

    outer_name = 'c_stator_outer';

    inner_name = 'c_stator_inner';

end

%% ------------------------------------------------------------------------
% Create stator ring
% -------------------------------------------------------------------------

dif_ring = geom.feature.create( ...
    'dif_ring', ...
    'Difference');

dif_ring.selection('input').set({outer_name});

dif_ring.selection('input2').set({inner_name});

%% ------------------------------------------------------------------------
% Slot parameters
% -------------------------------------------------------------------------

slot_pitch = 2*pi/Qs;

r_slot_outer = r_si + slot_depth;

%% ------------------------------------------------------------------------
% Copper parameters
% -------------------------------------------------------------------------

slot_fill_factor = 0.7;

copper_depth = slot_fill_factor * slot_depth;

copper_width = slot_fill_factor * slot_width;

r_copper_outer = r_si + copper_depth;

%% ------------------------------------------------------------------------
% Number of slots to draw
% -------------------------------------------------------------------------

if draw_only_sector

    n_slots_to_draw = round(Qs/(2*p));

else

    n_slots_to_draw = Qs;

end

slot_tags = cell(1,n_slots_to_draw);

%% ------------------------------------------------------------------------
% Create slots and copper regions
% -------------------------------------------------------------------------

for k = 1:n_slots_to_draw

    %% ---- Slot angle ---------------------------------------------------

    if draw_only_sector

        local_pitch = sector_angle / n_slots_to_draw;

        theta = (k-0.5)*local_pitch;

    else

        theta = (k-1)*slot_pitch;

    end

    %% ---- Slot geometry -----------------------------------------------

    theta1 = theta - slot_width/(2*r_si);

    theta2 = theta + slot_width/(2*r_si);

    theta3 = theta + slot_width/(2*r_slot_outer);

    theta4 = theta - slot_width/(2*r_slot_outer);

    x = [ ...
        r_si*cos(theta1), ...
        r_si*cos(theta2), ...
        r_slot_outer*cos(theta3), ...
        r_slot_outer*cos(theta4)];

    y = [ ...
        r_si*sin(theta1), ...
        r_si*sin(theta2), ...
        r_slot_outer*sin(theta3), ...
        r_slot_outer*sin(theta4)];

    tag = sprintf('slot%d',k);

    slot_tags{k} = tag;

    slot = geom.feature.create(tag,'Polygon');

    slot.set('x',x);

    slot.set('y',y);

    %% ---- Copper geometry ---------------------------------------------

    theta1_c = theta - copper_width/(2*r_si);

    theta2_c = theta + copper_width/(2*r_si);

    theta3_c = theta + copper_width/(2*r_copper_outer);

    theta4_c = theta - copper_width/(2*r_copper_outer);

    x_c = [ ...
        (r_si + 0.1*slot_depth)*cos(theta1_c), ...
        (r_si + 0.1*slot_depth)*cos(theta2_c), ...
        r_copper_outer*cos(theta3_c), ...
        r_copper_outer*cos(theta4_c)];

    y_c = [ ...
        (r_si + 0.1*slot_depth)*sin(theta1_c), ...
        (r_si + 0.1*slot_depth)*sin(theta2_c), ...
        r_copper_outer*sin(theta3_c), ...
        r_copper_outer*sin(theta4_c)];

    copper_tag = sprintf('copper%d',k);

    copper = geom.feature.create( ...
        copper_tag, ...
        'Polygon');

    copper.set('x',x_c);

    copper.set('y',y_c);

end

%% ------------------------------------------------------------------------
% Subtract slots from stator ring
% -------------------------------------------------------------------------

dif_slots = geom.feature.create( ...
    'dif_slots', ...
    'Difference');

dif_slots.selection('input').set({'dif_ring'});

dif_slots.selection('input2').set(slot_tags);

%% ------------------------------------------------------------------------
% Build geometry
% -------------------------------------------------------------------------

geom.run();

%% ------------------------------------------------------------------------
% CREATE EXPLICIT SELECTIONS
% -------------------------------------------------------------------------

sel_copper_up = model.selection.create( ...
    'sel_stator_copper_up', ...
    'Explicit');

sel_copper_up.label('Stator copper up');

sel_copper_down = model.selection.create( ...
    'sel_stator_copper_down', ...
    'Explicit');

sel_copper_down.label('Stator copper down');

%% ------------------------------------------------------------------------
% DOMAIN IDENTIFICATION
% -------------------------------------------------------------------------

all_domains = mphselectbox( ...
    model, ...
    geom_tag, ...
    [-r_so r_so; -r_so r_so], ...
    'domain');

all_domains = sort(all_domains);

%% ------------------------------------------------------------------------
% Stator iron domain
% -------------------------------------------------------------------------

iron_domain = min(all_domains);

%% ------------------------------------------------------------------------
% Copper domains
% -------------------------------------------------------------------------

copper_domains = setdiff( ...
    all_domains, ...
    iron_domain);

%% ------------------------------------------------------------------------
% Split copper UP / DOWN
% -------------------------------------------------------------------------

copper_up_domains = [];

copper_down_domains = [];

for k = 1:length(copper_domains)

    if mod(k,2)==1

        copper_up_domains = ...
            [copper_up_domains copper_domains(k)];

    else

        copper_down_domains = ...
            [copper_down_domains copper_domains(k)];

    end

end

%% ------------------------------------------------------------------------
% Apply copper selections
% -------------------------------------------------------------------------

sel_copper_up.geom(2);

sel_copper_up.set(copper_up_domains);

sel_copper_down.geom(2);

sel_copper_down.set(copper_down_domains);

%% ------------------------------------------------------------------------
% Console feedback
% -------------------------------------------------------------------------

fprintf('[draw_stator_sector] Geometry completed.\n');

fprintf('  Sector model enabled : %s\n', ...
    mat2str(draw_only_sector));

fprintf('  Slots drawn          : %d\n', ...
    n_slots_to_draw);

fprintf('  Copper regions added successfully.\n');

end