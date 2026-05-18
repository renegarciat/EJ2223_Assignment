classdef ComsolInterface < handle
   %ComsolInterface MATLAB wrapper for running COMSOL models via LiveLink.
   %
   % Primary entry points:
   %   - results = ComsolInterface.runComsolSimulation(params, config)
   %   - ci = ComsolInterface(config); results = ci.run(params)
   %   - ComsolInterface.start(config) (optional warm-up / handshake)

   properties (SetAccess = private)
      config
      model
      compTag
      geomTag
      physTag
      motorGeometry
      materialData
      leftMagnetPoints
      rightMagnetPoints
   end
   methods
      function obj = ComsolInterface(varargin)
         config = struct();
         geometry = [];
         materialData = MotorMaterials();

         % Supported call patterns:
         %   ComsolInterface()
         %   ComsolInterface(config)
         %   ComsolInterface(geometry)
         %   ComsolInterface(geometry, materials)
         %   ComsolInterface(config, geometry)
         %   ComsolInterface(config, geometry, materials)
         if nargin == 1
            if isa(varargin{1}, 'MotorGeometry')
               geometry = varargin{1};
            elseif isstruct(varargin{1})
               config = varargin{1};
            else
               error('Single input must be either config struct or MotorGeometry.');
            end
         elseif nargin == 2
            if isa(varargin{1}, 'MotorGeometry') && ComsolInterface.isMaterialContainer_(varargin{2})
               geometry = varargin{1};
               materialData = varargin{2};
            elseif isstruct(varargin{1}) && isa(varargin{2}, 'MotorGeometry')
               config = varargin{1};
               geometry = varargin{2};
            else
               error(['Two-input constructor supports only: ' ...
                      '(MotorGeometry, materialsStructOrObject) or (configStruct, MotorGeometry).']);
            end
         elseif nargin == 3
            if isstruct(varargin{1}) && isa(varargin{2}, 'MotorGeometry') && ComsolInterface.isMaterialContainer_(varargin{3})
               config = varargin{1};
               geometry = varargin{2};
               materialData = varargin{3};
            else
               error('Three-input constructor must be (configStruct, MotorGeometry, materialsStructOrObject).');
            end
         elseif nargin > 3
            error('Too many constructor inputs.');
         end

         projectRootDefault = ComsolInterface.defaultProjectRoot();
         cfg = ComsolInterface.applyDefaults(config, projectRootDefault);

         % Resolve model path early to fail fast (only for simulation runs).
         % if isempty(cfg.modelPath)
         %    cfg.modelPath = ComsolInterface.resolveModelPath(cfg.projectRoot);
         % end

         if ~isfield(cfg, 'compTag') || isempty(cfg.compTag)
            cfg.compTag = 'comp1';
         end
         if ~isfield(cfg, 'geomTag') || isempty(cfg.geomTag)
            cfg.geomTag = 'geom1';
         end
         if ~isfield(cfg, 'physTag') || isempty(cfg.physTag)
            cfg.physTag = 'mf';
         end

         obj.config = cfg;
         obj.compTag = cfg.compTag;
         obj.geomTag = cfg.geomTag;
         obj.physTag = cfg.physTag;
         obj.model = [];
         obj.leftMagnetPoints = [];
         obj.rightMagnetPoints = [];
         obj.materialData = ComsolInterface.applyMaterialDefaults(materialData);

         if ~isempty(geometry)
            obj.motorGeometry = geometry;
         end
      end

      function setMotorGeometry(obj, geometry)
         arguments
            obj
            geometry MotorGeometry
         end
         obj.motorGeometry = geometry;
      end

      function setMaterialData(obj, materialData)
         arguments
            obj
            materialData
         end
         if ~ComsolInterface.isMaterialContainer_(materialData)
            error('materialData must be a struct or MotorMaterials object.');
         end
         obj.materialData = ComsolInterface.applyMaterialDefaults(materialData);
      end

      function drawStatorSector(obj, geometry)
         if nargin >= 2
            obj.setMotorGeometry(geometry);
         end

         g = obj.requireMotorGeometry_();
         obj.ensureRawScriptPath_();
         obj.ensureModelReady_();

         draw_stator_sector(obj.model, obj.geomTag, ...
                           g.StatorInnerRadius_m, ...
                           g.StatorOuterRadius_m, ...
                           g.Slots, ...
                           g.PolePairs, ...
                           g.SlotDepth_m, ...
                           g.SlotWidth_m, ...
                           g.DrawOnlySector);
      end

      function drawRotorSector(obj, geometry)
         if nargin >= 2
            obj.setMotorGeometry(geometry);
         end

         g = obj.requireMotorGeometry_();
         obj.ensureRawScriptPath_();
         obj.ensureModelReady_();

         [obj.leftMagnetPoints, obj.rightMagnetPoints] = draw_rotor_sector(obj.model, obj.geomTag, ...
                           g.RotorOuterRadius_m, ...
                           g.RotorInnerRadius_m, ...
                           g.Airgap_m, ...
                           g.PolePairs, ...
                           g.MagnetLength_m, ...
                           g.MagnetWidth_m, ...
                           g.MagnetSpacing_m, ...
                           g.MagnetRibHeight_m, ...
                           g.MagnetAngle_rad, ...
                           g.DrawOnlySector);
      end

      function createSelections(obj, geometry)
         if nargin >= 2
            obj.setMotorGeometry(geometry);
         end

         g = obj.requireMotorGeometry_();
         obj.ensureRawScriptPath_();
         obj.ensureModelReady_();

         if isempty(obj.leftMagnetPoints) || isempty(obj.rightMagnetPoints)
            error('Rotor magnet points are missing. Call drawRotorSector before createSelections.');
         end

         create_selections(obj.model, obj.geomTag, g.DrawOnlySector, ...
                           obj.leftMagnetPoints, obj.rightMagnetPoints, ...
                           g.RotorInnerRadius_m, ...
                           g.RotorOuterRadius_m, ...
                           g.StatorInnerRadius_m, ...
                           g.StatorOuterRadius_m, ...
                           g.PolePairs);
      end

      function defineMaterials(obj, materialData, geometry)
         if nargin >= 2
            obj.setMaterialData(materialData);
         end
         if nargin >= 3
            obj.setMotorGeometry(geometry);
         end

         g = obj.requireMotorGeometry_();
         obj.ensureRawScriptPath_();
         obj.ensureModelReady_();

         mats = ComsolInterface.applyMaterialDefaults(obj.materialData);

         define_materials(obj.model, obj.compTag, obj.physTag, g.DrawOnlySector, mats.mesh_size, ...
                          mats.mu_r_shaft, mats.sigma_shaft, mats.epsilon_r_shaft, ...
                          mats.mu_r_iron, mats.sigma_iron, mats.epsilon_r_iron, ...
                          mats.mu_r_air, mats.sigma_air, ...
                          mats.mu_r_magnets, mats.sigma_magnets, mats.Br);
      end

      function saveModel(obj, savePath)
         arguments
            obj
            savePath (1,:) char
         end
         if isempty(obj.model)
            error('No COMSOL model exists to save. Call drawStatorSector/drawRotorSector first.');
         end
         obj.model.save(savePath);
      end

      function results = run(obj, params)
         %RUN Execute the full COMSOL workflow (load model, push params, run study, extract results).
         if nargin < 2
            params = struct();
         end

         cfg = obj.config;
         results = struct();

         % Old behavior: if params not provided, compute simple sizing defaults.
         if isempty(fieldnames(params))
            req = ComsolInterface.makeRequirements();
            design = ComsolInterface.computeDesign(req);
            params = ComsolInterface.makeComsolParams(design);
            results.requirements = req;
            results.design = design;
         end

         % ---- COMSOL LiveLink path ----
         comsolMliDir = ComsolInterface.resolveComsolMliDir(cfg);
         if ~isempty(comsolMliDir)
            if exist(comsolMliDir, 'dir') ~= 7
               error('COMSOL MLI directory not found: %s', comsolMliDir);
            end
            addpath(comsolMliDir);
         end

         % ---- Initialize COMSOL connection ----
         ComsolInterface.startComsol();

         % ---- Load model ----
            fprintf('Loading COMSOL model: %s...\n', cfg.modelPath);
            mdl = ComsolInterface.loadModel(cfg.modelPath);
         fprintf('Model loaded successfully.\n');

         % ---- Update model parameters ----
         fprintf('Updating COMSOL parameters...\n');
         ComsolInterface.setComsolParams(mdl, params);

         % Java progress bar (non-fatal if not available)
         ComsolInterface.enableComsolProgress();

         % ---- Run simulation ----
         fprintf('Starting COMSOL Solver... this may take a few minutes.\n');
         tic;
            ComsolInterface.runStudy(mdl, cfg.studyTag);
         simTime = toc;
         fprintf('Simulation complete. Time: %.2f seconds.\n', simTime);

         % ---- Retrieve results ----
            [t, torque_raw] = ComsolInterface.extractTorqueFromTable(mdl, cfg.resultTableTag);

         avg_torque = mean(torque_raw);
         if avg_torque == 0
            torque_ripple = NaN;
         else
            torque_ripple = (max(torque_raw) - min(torque_raw)) / avg_torque * 100;
         end

         fprintf('--- Results ---\n');
         fprintf('Average Torque: %.2f Nm\n', avg_torque);
         fprintf('Torque Ripple: %.2f %%\n', torque_ripple);

         if cfg.makePlot
            figure(1);
            plot(t, torque_raw, 'LineWidth', 1.5, 'Color', [0 0.447 0.741]);
            grid on;
            title('IPM Performance: Torque vs Time');
            xlabel('Time (s)');
            ylabel('Torque (Nm)');
            legend('FEA Result');
         end

         % ---- Pack results ----
         results.avg_torque = avg_torque;
         results.ripple = torque_ripple;
         results.t = t;
         results.torque = torque_raw;
         results.meta.timestamp = datestr(now, 31);
         results.meta.modelPath = cfg.modelPath;
         results.meta.studyTag = cfg.studyTag;
         results.meta.resultTableTag = cfg.resultTableTag;
         results.meta.simTime_s = simTime;
         results.meta.paramsPushed = sort(fieldnames(params));
         results.params = params;

         if cfg.saveResults
            save(cfg.savePath, 'results');
         end
      end
   end

   methods (Static)
      function results = runComsolSimulation(params, config)
         %RUNCOMSOLSIMULATION Convenience static wrapper.
         if nargin < 1
            params = struct();
         end
         if nargin < 2
            config = struct();
         end

         ci = ComsolInterface(config);
         results = ci.run(params);
      end

      function start(config)
         %START Optional: add MLI path and connect to COMSOL server.
         if nargin < 1
            config = struct();
         end
         projectRootDefault = ComsolInterface.defaultProjectRoot();
         cfg = ComsolInterface.applyDefaults(config, projectRootDefault);

         comsolMliDir = ComsolInterface.resolveComsolMliDir(cfg);
         if ~isempty(comsolMliDir)
            if exist(comsolMliDir, 'dir') ~= 7
               error('COMSOL MLI directory not found: %s', comsolMliDir);
            end
            addpath(comsolMliDir);
         end

         ComsolInterface.startComsol();
      end
   end

   methods (Static, Access = private)
      function mats = applyMaterialDefaults(materialData)
         mats = struct();
         if nargin < 1
            materialData = MotorMaterials();
         end

         if ~ComsolInterface.isMaterialContainer_(materialData)
            error('materialData must be a struct or MotorMaterials object.');
         end

         mats.mesh_size = ComsolInterface.pickField_(materialData, 'mesh_size', 5);

         mats.mu_r_shaft = ComsolInterface.pickField_(materialData, 'mu_r_shaft', 1);
         mats.sigma_shaft = ComsolInterface.pickField_(materialData, 'sigma_shaft', 1.4e6);
         mats.epsilon_r_shaft = ComsolInterface.pickField_(materialData, 'epsilon_r_shaft', 0.8);

         mats.mu_r_iron = ComsolInterface.pickField_(materialData, 'mu_r_iron', 5000);
         mats.sigma_iron = ComsolInterface.pickField_(materialData, 'sigma_iron', 2e6);
         mats.epsilon_r_iron = ComsolInterface.pickField_(materialData, 'epsilon_r_iron', 0.8);

         mats.mu_r_air = ComsolInterface.pickField_(materialData, 'mu_r_air', 1);
         mats.sigma_air = ComsolInterface.pickField_(materialData, 'sigma_air', 0);

         mats.mu_r_magnets = ComsolInterface.pickField_(materialData, 'mu_r_magnets', 1.05);
         mats.sigma_magnets = ComsolInterface.pickField_(materialData, 'sigma_magnets', 6.25e5);
         mats.Br = ComsolInterface.pickField_(materialData, 'Br', 1.3);
      end

      function value = pickField_(s, fieldName, defaultValue)
         if isstruct(s)
            hasValue = isfield(s, fieldName) && ~isempty(s.(fieldName));
         else
            hasValue = isprop(s, fieldName) && ~isempty(s.(fieldName));
         end

         if hasValue
            value = s.(fieldName);
         else
            value = defaultValue;
         end
      end

      function tf = isMaterialContainer_(value)
         tf = isstruct(value) || isa(value, 'MotorMaterials');
      end

      function projectRoot = defaultProjectRoot()
         classFile = which('ComsolInterface');
         if isempty(classFile)
            % Fallback: current directory.
            projectRoot = pwd;
            return;
         end
         thisDir = fileparts(classFile);
         projectRoot = fullfile(thisDir, '..');
      end

      function config = applyDefaults(config, projectRootDefault)
         if ~isfield(config, 'projectRoot') || isempty(config.projectRoot)
            config.projectRoot = projectRootDefault;
         end
         if ~isfield(config, 'modelPath')
            config.modelPath = '';
         end
         if ~isfield(config, 'studyTag') || isempty(config.studyTag)
            config.studyTag = 'std1';
         end
         if ~isfield(config, 'resultTableTag') || isempty(config.resultTableTag)
            config.resultTableTag = 'tbl1';
         end
         if ~isfield(config, 'saveResults')
            config.saveResults = true;
         end
         if ~isfield(config, 'savePath') || isempty(config.savePath)
            config.savePath = fullfile(config.projectRoot, 'Simulation_Results.mat');
         end
         if ~isfield(config, 'makePlot')
            config.makePlot = true;
         end
         if ~isfield(config, 'comsolMliDir')
            config.comsolMliDir = '';
         end
         if ~isfield(config, 'compTag')
            config.compTag = 'comp1';
         end
         if ~isfield(config, 'geomTag')
            config.geomTag = 'geom1';
         end
         if ~isfield(config, 'physTag')
            config.physTag = 'mf';
         end
      end

      function comsolMliDir = resolveComsolMliDir(config)
         if isfield(config, 'comsolMliDir') && ~isempty(config.comsolMliDir)
            comsolMliDir = config.comsolMliDir;
            return;
         end

         comsolMliDir = getenv('COMSOL_MLI_DIR');
         if ~isempty(comsolMliDir)
            return;
         end

         % No explicit override provided; try OS-specific common install paths.
         candidates = {};

         if ispc
            % Common Windows layout: C:\Program Files\COMSOL\COMSOL6x\Multiphysics\mli
            programFiles = {getenv('PROGRAMFILES'), getenv('PROGRAMFILES(X86)')};
            programFiles = programFiles(~cellfun(@isempty, programFiles));
            programFiles = unique(programFiles, 'stable');

            for i = 1:numel(programFiles)
               base = programFiles{i};
               % Prefer discovering actual installed versions.
               matches = dir(fullfile(base, 'COMSOL', 'COMSOL*', 'Multiphysics', 'mli'));
               % If multiple exist, try newest-looking first.
               % Older MATLAB versions don't support sort(...,'descend') for cell arrays.
               [~, order] = sort({matches.name});
               order = order(end:-1:1);
               matches = matches(order);
               for k = 1:numel(matches)
                  if matches(k).isdir
                     candidates{end+1} = fullfile(matches(k).folder, matches(k).name); %#ok<AGROW>
                  end
               end
            end

            % Fallback guesses (in case COMSOL folder isn't enumerable).
            if isempty(candidates) && ~isempty(getenv('PROGRAMFILES'))
               pf = getenv('PROGRAMFILES');
               candidates = {
                  fullfile(pf, 'COMSOL', 'COMSOL62', 'Multiphysics', 'mli')
                  fullfile(pf, 'COMSOL', 'COMSOL61', 'Multiphysics', 'mli')
                  fullfile(pf, 'COMSOL', 'COMSOL60', 'Multiphysics', 'mli')
               };
            end

         elseif ismac
            % Typical macOS application bundle layout (may vary by version).
            matches = dir('/Applications/COMSOL*.app/Contents/Resources/mli');
            % Older MATLAB versions don't support sort(...,'descend') for cell arrays.
            [~, order] = sort({matches.name});
            order = order(end:-1:1);
            matches = matches(order);
            for k = 1:numel(matches)
               if matches(k).isdir
                  candidates{end+1} = fullfile(matches(k).folder, matches(k).name); %#ok<AGROW>
               end
            end

            % Also try Unix-like locations if installed differently.
            candidates = [candidates(:); {
               '/usr/local/opt/comsol64/multiphysics/mli'
               '/opt/comsol64/multiphysics/mli'
            }];

         else
            % Linux / other Unix
            preferred = '/home/renet/opt/comsol64/multiphysics/mli';
            if exist(preferred, 'dir') == 7
               comsolMliDir = preferred;
               return;
            end

            homeDir = getenv('HOME');
            if ~isempty(homeDir)
               candidates{end+1} = fullfile(homeDir, 'opt', 'comsol64', 'multiphysics', 'mli');
            end

            candidates = [candidates(:); {
               '/usr/local/opt/comsol64/multiphysics/mli'
               '/opt/comsol64/multiphysics/mli'
               '/usr/local/comsol64/multiphysics/mli'
            }];

            matches = [
               dir('/usr/local/opt/comsol*/multiphysics/mli');
               dir('/opt/comsol*/multiphysics/mli');
               dir('/usr/local/comsol*/multiphysics/mli')
            ];
            % Older MATLAB versions don't support sort(...,'descend') for cell arrays.
            [~, order] = sort({matches.name});
            order = order(end:-1:1);
            matches = matches(order);
            for k = 1:numel(matches)
               if matches(k).isdir
                  candidates{end+1} = fullfile(matches(k).folder, matches(k).name); %#ok<AGROW>
               end
            end
         end

         for i = 1:numel(candidates)
            if exist(candidates{i}, 'dir') == 7
               comsolMliDir = candidates{i};
               return;
            end
         end

         % Last-resort fallback (will likely trigger the caller's existence check)
         if ispc
            comsolMliDir = fullfile(getenv('PROGRAMFILES'), 'COMSOL', 'COMSOL62', 'Multiphysics', 'mli');
         elseif ismac
            comsolMliDir = '/Applications/COMSOL.app/Contents/Resources/mli';
         else
            comsolMliDir = '/usr/local/opt/comsol64/multiphysics/mli';
         end
      end

      function req = makeRequirements()
         req = struct();

         % Targets
         req.T = 17;               % Torque target (Nm)
         req.D = 0.08;             % Rotor outer diameter (m)
         req.C0 = 45000;           % Output Coefficient (Ws/m^3)
         req.windingHead = 0.0238; % Winding head length added to active length (m)

         % Constraints
         req.D_MAX = 0.08;                      % Max rotor outer diameter (m)
         req.L_MAX = 0.061 + 0.0124 + 0.0118;   % Max total length (m)
      end

      function design = computeDesign(req)
         design = struct();

         design.L_active = req.T / (req.C0 * req.D^2);
         design.L = design.L_active + req.windingHead;

         fprintf('Torque (T): %.2f Nm\n', req.T);
         fprintf('Rotor Outer Diameter (D): %.2f m\n', req.D);
         fprintf('Output Coefficient (C0): %.2f Ws/m^3\n', req.C0);
         fprintf('Calculated Stack Length (L): %.4f m\n', design.L);

         ComsolInterface.assert_requirements(req, design);
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
         params.L = sprintf('%.4f[m]', design.L);

         % Operating point / excitation
         params.w_rot = '12000[rpm]';
         params.Ipk = '107[A]';
         params.init_ang = '200[deg]';

         % V-shape IPM Specifics
         params.w_m = '14[mm]';
         params.t_m = '3.5[mm]';
         params.alpha_v = '130[deg]';
         params.b_t = '1.0[mm]';

         if isfield(params, 't')
            params = rmfield(params, 't');
         end
      end

      function modelPath = resolveModelPath(projectRoot)
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

      function startComsol()
         try
            fprintf('Checking for COMSOL Server connection...\n');
            mphstart;
            fprintf('Successfully connected to COMSOL Server.\n');
         catch ME
            if contains(ME.message, 'Already connected')
               fprintf('COMSOL connection already active. Skipping handshake.\n');
            elseif contains(ME.message, 'Connection refused') || contains(ME.message, 'ConnectException')
               error('[COMSOL ERROR]: Server not found. Run "comsol mphserver" in terminal.');
            else
               error('Unexpected connection error: %s', ME.message);
            end
         end
      end

      function model = loadModel(modelPath)
         try
            model = mphload(modelPath);
         catch ME
            error(['Could not find or load the file: %s. ' ...
                  'Ensure the .mph file exists and COMSOL can access it. ' ...
                  'Error: %s'], modelPath, ME.message);
         end
      end

      function enableComsolProgress()
         try
            com.comsol.model.util.ModelUtil.showProgress(true);
         catch
            % Non-fatal; some setups restrict Java imports in MATLAB.
         end
      end

      function setComsolParams(model, params)
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
            studyTags = ComsolInterface.safeTags(@() model.study.tags);
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
            tableTags = ComsolInterface.safeTags(@() model.result.table.tags);
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
   end

   methods (Access = private)
      function ensureRawScriptPath_(obj)
         rawScriptsPath = fullfile(obj.config.projectRoot, 'MATLAB_Code');
         if exist(rawScriptsPath, 'dir') ~= 7
            error('Raw COMSOL script folder not found: %s', rawScriptsPath);
         end
         addpath(rawScriptsPath);
      end

      function ensureModelReady_(obj)
         if ~isempty(obj.model)
            return;
         end

         ComsolInterface.start(obj.config);
         import com.comsol.model.*
         import com.comsol.model.util.*

         obj.model = ModelUtil.create('MotorModel');
         obj.model.component.create(obj.compTag, true);
         obj.model.component(obj.compTag).geom.create(obj.geomTag, 2);
         obj.model.component(obj.compTag).physics.create(obj.physTag, 'InductionCurrents', obj.geomTag);
      end

      function g = requireMotorGeometry_(obj)
         if isempty(obj.motorGeometry)
            error('MotorGeometry is not set. Pass a MotorGeometry object or call setMotorGeometry first.');
         end
         g = obj.motorGeometry;
      end
   end
end