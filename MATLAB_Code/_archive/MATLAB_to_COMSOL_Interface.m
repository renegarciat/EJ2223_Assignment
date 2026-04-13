% 1. Create script to use the equations in the article. Hadrien. 
% 2. Create functions.
% 3. Integrate the geometry drawing into the script.
% 4. Results: Establish different sets of simulations. 1. Magnetostatic simulation. Obtain torque 
% out of the current. Evaluate the torque ripple by ... The rotor side one rotates it a little bit and
% re-run the simulation to get the torque. After an entire rotation, get the plot toruqe ripple vs. position.
% Back EMF: Get flux linkage in the windings as we expect. Get the efficiency.  3 things.
% Call amount of degrees. Call using a for loop. 
% 
%% KTH Formula Student - IPM Motor Design Automation
% Author: IPM Design Team
% Description: Modular script to interface MATLAB with COMSOL for 
% IPM motor performance validation (Torque & Ripple).
% with Username: admin and Password: admin before executing this script.
clear; clc;

% NOTE:
% The COMSOL runner has been refactored into a callable function:
%   run_comsol_simulation.m
% For the combined workflow (Essen sizing -> COMSOL), use:
%   MATLAB_Code/main.m
run('main.m');
return;

%% 1. CONFIGURATION (tailored to the prebuilt IPM model)
% This script is meant to drive the provided COMSOL model
%   pm_motor_2d_introduction.mph
% by setting the model's Global Parameters directly from this file.

% (A) COMSOL LiveLink (MLI) path
% Prefer setting an environment variable COMSOL_MLI_DIR, e.g.
%   export COMSOL_MLI_DIR=/path/to/comsol/multiphysics/mli
comsolMliDir = getenv('COMSOL_MLI_DIR');
if isempty(comsolMliDir)
    comsolMliDir = '/home/renet/opt/comsol64/multiphysics/mli';
end
addpath(comsolMliDir);

% (B) Resolve project paths relative to this script
thisFile = mfilename('fullpath');
thisDir = fileparts(thisFile);
projectRoot = fullfile(thisDir, '..');

config.modelPath = resolveModelPath(projectRoot);

% (C) COMSOL tags (adjust if your model uses different tags)
config.studyTag = 'std1';
config.resultTableTag = 'tbl1';

% (D) Requirements / sizing (integrated from MATLAB_Code/main.m)
req = makeRequirements();
design = computeDesign(req);

% (D) Single source of truth for COMSOL parameters
% NOTE: Values are COMSOL expressions (strings). Include units where appropriate.
% Names must match the model's Global Parameters.
params = makeComsolParams(design);


%% 2. INITIALIZE COMSOL CONNECTION
try
    fprintf('Checking for COMSOL Server connection...\n');
    mphstart; 
    fprintf('Successfully connected to COMSOL Server.\n');
catch ME
    if contains(ME.message, 'Already connected')
        fprintf('COMSOL connection already active. Skipping handshake.\n');
    elseif contains(ME.message, 'Connection refused') || contains(ME.message, 'ConnectException')
        error(['[COMSOL ERROR]: Server not found. Run "comsol mphserver" in terminal.']);
    else
        error(['Unexpected connection error: ' ME.message]);
    end
end

%% 2.1 LOAD THE MOTOR MODEL
try
    fprintf('Loading COMSOL model: %s...\n', config.modelPath);
    model = mphload(config.modelPath);
    fprintf('Model loaded successfully.\n');
catch ME
    error(['Could not find or load the file: %s. ' ...
           'Ensure the .mph file exists and COMSOL can access it. ' ...
           'Error: %s'], config.modelPath, ME.message);
end

%% 3. UPDATE MODEL PARAMETERS
fprintf('Updating COMSOL parameters...\n');
setComsolParams(model, params);

% Initialize Java classes (for progress bar)
try
    import com.comsol.model.util.*
    ModelUtil.showProgress(true);
catch
    % Non-fatal; some setups restrict Java imports in MATLAB.
end

%% 4. RUN SIMULATION (Phase 2: 2D Magnetic FEA)
fprintf('Starting COMSOL Solver... this may take a few minutes.\n');
tic;
runStudy(model, config.studyTag);
simTime = toc;
fprintf('Simulation complete. Time: %.2f seconds.\n', simTime);

%% 5. RETRIEVE RESULTS & POST-PROCESSING
% Torque extraction is model-dependent. Default approach is a results table.
[t, torque_raw] = extractTorqueFromTable(model, config.resultTableTag);

% Calculations for your report
avg_torque = mean(torque_raw);
torque_ripple = (max(torque_raw) - min(torque_raw)) / avg_torque * 100;

fprintf('--- Results ---\n');
fprintf('Average Torque: %.2f Nm\n', avg_torque);
fprintf('Torque Ripple: %.2f %%\n', torque_ripple);

%% 6. VISUALIZATION
figure(1);
plot(t, torque_raw, 'LineWidth', 1.5, 'Color', [0 0.447 0.741]);
grid on;
title('IPM Performance: Torque vs Time');
xlabel('Time (s)');
ylabel('Torque (Nm)');
legend('FEA Result');

% Save results to a structure for later comparison
results.avg_torque = avg_torque;
results.ripple = torque_ripple;
results.t = t;
results.torque = torque_raw;
results.meta.timestamp = datestr(now, 31);
results.meta.modelPath = config.modelPath;
results.meta.studyTag = config.studyTag;
results.meta.resultTableTag = config.resultTableTag;
results.meta.simTime_s = simTime;
results.meta.paramsPushed = sort(fieldnames(params));
results.params = params;
results.requirements = req;
results.design = design;
save(fullfile(projectRoot, 'Simulation_Results.mat'), 'results');

%% ---- Local functions ----

function req = makeRequirements()
% Requirements / constraints
    req = struct();

    % Targets
    req.T = 17;              % Torque target (Nm)
    req.D = 0.08;            % Rotor outer diameter (m)
    req.C0 = 45000;          % Target Output Coefficient (Ws/m^3)
    req.windingHead = 0.0238; % Winding head length added to active length (m)

    % Constraints
    req.D_MAX = 0.08;                           % Maximum rotor outer diameter (m)
    req.L_MAX = 0.061 + 0.0124 + 0.0118;       % Maximum stack length (m)
end

function design = computeDesign(req)
% Computes stack length from sizing formula and checks constraints.
    design = struct();

    design.L_active = req.T / (req.C0 * req.D^2);
    design.L = design.L_active + req.windingHead;

    fprintf('Torque (T): %.2f Nm\n', req.T);
    fprintf('Rotor Outer Diameter (D): %.2f m\n', req.D);
    fprintf('Output Coefficient (C0): %.2f Ws/m^3\n', req.C0);
    fprintf('Calculated Stack Length (L): %.4f m\n', design.L);

    assert_requirements(req, design);
end

function assert_requirements(req, design)
    assert(design.L <= req.L_MAX, ...
        'The required stack length (%.4f m) exceeds the maximum allowed length: %.4f m.', ...
        design.L, req.L_MAX);
    assert(req.D <= req.D_MAX, ...
        'The required outer diameter (%.4f m) exceeds the maximum allowed diameter: %.4f m.', ...
        req.D, req.D_MAX);
end

function params = makeComsolParams(design)
% Edit this function only; it is the single source of truth for what MATLAB
% pushes into the COMSOL model.
%
% Optimal pattern: MATLAB sets only independent inputs (geometry + operating
% point). Derived quantities (electrical frequency, time stepping, ramps,
% etc.) should be defined inside the COMSOL model as expressions.

    params = struct();

    % Geometry
    params.Np = '10';
    params.Ns = '12';
    params.mag_h = '2.5[mm]';
    params.d_s = '10[mm]';
    params.d_r = '80[mm]';
    params.d_st = '100[mm]';
    params.airgap = '0.7[mm]';
    params.d_cont = '80.7[mm]';
    params.L = sprintf('%.4f[m]', design.L); % Stack length (m) - sized from requirements

    % Operating point / excitation
    params.w_rot = '12000[rpm]';
    params.Ipk = '107[A]';
    params.init_ang = '200[deg]';

    % V-shape IPM Specifics
    params.w_m = '14[mm]'; % Magnetic Slab Width 
    params.t_m = '3.5[mm]';  % Magnet Thickness
    params.alpha_v = '130[deg]'; % V angle between magnets
    params.b_t = '1.0[mm]'; % Outer iron bridge Thickness
    % Avoid setting COMSOL's built-in time variable name.
    if isfield(params, 't')
        params = rmfield(params, 't');
    end
end

function modelPath = resolveModelPath(projectRoot)
% Prefer the non-autosaved model in the assignment root, but fall back to
% autosaved copies if needed.

    candidates = {
        fullfile(projectRoot, 'COMSOL_models', 'pm_motor_2d_introduction.mph')
    };

    for i = 1:numel(candidates)
        if exist(candidates{i}, 'file')
            modelPath = candidates{i};
            return;
        end
    end

    found = dir(fullfile(projectRoot, '**', '*.mph'));
    if isempty(found)
        foundList = '(none found)';
    else
        foundList = strjoin(fullfile({found.folder}, {found.name}), newline);
    end

    triedList = strjoin(candidates, newline);
    error(['No COMSOL model (.mph) found in expected locations.\n' ...
           'Tried:\n%s\n\n' ...
           'Discovered .mph files under project root:\n%s'], triedList, foundList);
end

function setComsolParams(model, params)
% params can be either:
%   - Nx2 cell array: { 'name','expr'; ... }
%   - containers.Map(name -> expr)
%   - struct with fields (fieldname -> expr)

    if iscell(params)
        if size(params, 2) ~= 2
            error('Parameter list must be an Nx2 cell array of {name, expression}.');
        end
        names = params(:, 1);
        values = params(:, 2);
    elseif isa(params, 'containers.Map')
        names = params.keys;
        values = cell(size(names));
        for i = 1:numel(names)
            values{i} = params(names{i});
        end
    elseif isstruct(params)
        names = fieldnames(params);
        values = cell(size(names));
        for i = 1:numel(names)
            values{i} = params.(names{i});
        end
    else
        error('Unsupported parameter container type: %s', class(params));
    end

    for i = 1:numel(names)
        name = strtrim(names{i});
        value = strtrim(values{i});

        if isempty(value)
            continue;
        end
        try
            mphsetparam(model, name, value);
        catch ME
            error(['Failed setting COMSOL parameter "%s" to "%s". ' ...
                   'Either the model uses a different name, or the expression is invalid.\n' ...
                   'Original error: %s'], name, value, ME.message);
        end
    end
end

function runStudy(model, studyTag)
    try
        model.study(studyTag).run;
    catch ME
        studyTags = safeTags(@() model.study.tags);
        if isempty(studyTags)
            hint = 'Could not list study tags. Open the model and confirm the study tag.';
        else
            hint = sprintf('Available studies: %s', strjoin(studyTags, ', '));
        end
        error('Failed running study "%s". %s\nOriginal error: %s', studyTag, hint, ME.message);
    end
end

function [t, torque] = extractTorqueFromTable(model, tableTag)
    try
        tbl = mphtable(model, tableTag);
    catch ME
        tableTags = safeTags(@() model.result.table.tags);
        if isempty(tableTags)
            hint = 'Could not list result tables. Create a Results -> Table with time and torque.';
        else
            hint = sprintf('Available tables: %s', strjoin(tableTags, ', '));
        end
        error('Failed reading results table "%s". %s\nOriginal error: %s', tableTag, hint, ME.message);
    end

    if ~isfield(tbl, 'data') || isempty(tbl.data) || size(tbl.data, 2) < 2
        error('Table "%s" did not contain numeric data with at least 2 columns.', tableTag);
    end

    % Heuristic: prefer columns whose headers contain "time" and "torque".
    timeCol = 1;
    torqueCol = 2;

    headers = {};
    if isfield(tbl, 'colheaders')
        headers = tbl.colheaders;
    end
    if isstring(headers)
        headers = cellstr(headers);
    end

    if iscell(headers) && ~isempty(headers)
        lowerHeaders = lower(strtrim(headers));
        idxTime = find(contains(lowerHeaders, 'time') | strcmp(lowerHeaders, 't'), 1, 'first');
        idxTorque = find(contains(lowerHeaders, 'torque') | contains(lowerHeaders, 'tem') | contains(lowerHeaders, 'mz') | contains(lowerHeaders, 'tz'), 1, 'first');
        if ~isempty(idxTime)
            timeCol = idxTime;
        end
        if ~isempty(idxTorque)
            torqueCol = idxTorque;
        end
    end

    t = tbl.data(:, timeCol);
    torque = tbl.data(:, torqueCol);
end

function tags = safeTags(getTagsFn)
% Calls a tag getter and converts to a cellstr if possible.
    try
        raw = getTagsFn();
    catch
        tags = {};
        return;
    end

    try
        if isstring(raw)
            tags = cellstr(raw);
        elseif ischar(raw)
            tags = cellstr(raw);
        elseif iscell(raw)
            tags = raw;
        else
            tags = cellstr(raw);
        end
    catch
        tags = {};
    end
end