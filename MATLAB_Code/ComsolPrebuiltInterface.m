classdef ComsolPrebuiltInterface < handle
   %ComsolPrebuiltInterface Push design values into the prebuilt AC/DC
   %Application Library IPM model. The model's geometry is already
   %expressed through the AC/DC Module's Rotating Machinery 2D Part
   %Library (Part Instances "pi1" = rotor, "pi2" = stator, see
   %references/models.acdc.interior_pm_motor_stress_analysis.pdf), whose
   %inputs mostly reference named Global Parameters (Np, Ns, d_s, d_r,
   %d_cont, mag_h, d_st, airgap, L, ...). This class only needs to push
   %new values into those parameters and rebuild the geometry.
   %
   % Template vs. output: TemplatePath (default
   % COMSOL_models/interior_pm_motor_stress_analysis_original.mph) is
   % read-only -- this class never writes to it, so it stays a clean,
   % correctly-wired baseline no matter how many times you push new
   % sizing results. Each run instead loads a fresh in-memory copy from
   % the template and, if you save, writes to OutputPath (default
   % COMSOL_models/interior_pm_motor_stress_analysis.mph). saveModelAs()
   % refuses to write to TemplatePath even if asked explicitly.
   %
   % Connection: connects to a running "comsol mphserver" (LiveLink
   % client-server) via ensureComsolConnected_(), then mphload()s its own
   % independent copy of the model into the server engine. This does NOT
   % touch a COMSOL Desktop GUI session that may have the template file
   % open -- mphload only reads the file from disk into a separate
   % in-memory model instance.
   %
   % Usage:
   %   ci = ComsolPrebuiltInterface();
   %   ci.pushGeometry(Np=10, Ns=12, Airgap_mm=1.0, StatorYokeHeight_mm=3.97, ...
   %                   RotorDiameter_mm=50.15, L_mm=40);
   %   ci.rebuildGeometry();
   %   ci.summary();
   %   ci.saveModelAs();   % writes to OutputPath; TemplatePath is never touched

   properties (SetAccess = private)
      templatePath   % read-only source; always freshly loaded, never written to
      outputPath     % default saveModelAs() destination
      compTag
      geomTag
      rotorPartTag   % pi1, "Internal Rotor - V-shaped Embedded Magnets"
      statorPartTag  % pi2, "External Stator - Slotted"
      meshTag        % mesh1
      model
   end

   methods
      function obj = ComsolPrebuiltInterface(options)
         arguments
            options.TemplatePath (1,:) char = ''
            options.OutputPath (1,:) char = ''
            options.CompTag (1,:) char = 'comp1'
            options.GeomTag (1,:) char = 'geom1'
            options.RotorPartTag (1,:) char = 'pi1'
            options.StatorPartTag (1,:) char = 'pi2'
            options.MeshTag (1,:) char = 'mesh1'
         end

         modelsDir = fullfile(ComsolPrebuiltInterface.defaultProjectRoot(), 'COMSOL_models');

         if isempty(options.TemplatePath)
            obj.templatePath = fullfile(modelsDir, 'interior_pm_motor_stress_analysis_original.mph');
         else
            obj.templatePath = options.TemplatePath;
         end

         if isempty(options.OutputPath)
            obj.outputPath = fullfile(modelsDir, 'interior_pm_motor_stress_analysis.mph');
         else
            obj.outputPath = options.OutputPath;
         end

         obj.compTag = options.CompTag;
         obj.geomTag = options.GeomTag;
         obj.rotorPartTag = options.RotorPartTag;
         obj.statorPartTag = options.StatorPartTag;
         obj.meshTag = options.MeshTag;
         obj.model = [];
      end

      function pushGeometry(obj, options)
         %pushGeometry  Set one or more of the eight design values as
         %COMSOL Global Parameters, then re-pin the Part Instance inputs
         %that are known to need a matching symbolic reference (rather
         %than trusting whatever literal value they were last hand-set
         %to in COMSOL Desktop). Any value not supplied is left alone.
         arguments
            obj
            options.Np (1,1) double
            options.Ns (1,1) double
            options.Airgap_mm (1,1) double
            options.StatorYokeHeight_mm (1,1) double
            options.RotorDiameter_mm (1,1) double
            options.MagnetHeight_mm (1,1) double
            options.L_mm (1,1) double
            options.MaxSpeed_rpm (1,1) double
         end

         obj.ensureModelReady_();

         if isfield(options, 'Np')
            obj.setParam_('Np', sprintf('%d', options.Np));
            obj.setPartInput_(obj.rotorPartTag,  'number_of_poles',         'Np');
            obj.setPartInput_(obj.rotorPartTag,  'number_of_modeled_poles', 'Np');
         end
         if isfield(options, 'Ns')
            obj.setParam_('Ns', sprintf('%d', options.Ns));
            obj.setPartInput_(obj.statorPartTag, 'number_of_slots',         'Ns');
            obj.setPartInput_(obj.statorPartTag, 'number_of_modeled_slots', 'Ns');
         end
         if isfield(options, 'Airgap_mm')
            % airgap has no direct Part Instance input -- it only feeds
            % other expressions (d_cont = d_r+airgap, tooth_h, ...), so
            % setting the global parameter is enough.
            obj.setParam_('airgap', obj.lengthStr_(options.Airgap_mm));
         end
         if isfield(options, 'RotorDiameter_mm')
            obj.setParam_('d_r', obj.lengthStr_(options.RotorDiameter_mm));
            obj.setPartInput_(obj.rotorPartTag, 'rotor_diam', 'd_r');
         end
         if isfield(options, 'MagnetHeight_mm')
            % mag_h ("Magnet height") already exists as a Global Parameter
            % and pi1's magnet_h input is already wired to it in the
            % template. Re-pin the Part Instance input too, same
            % defensive pattern as rotor_diam.
            %
            % NOTE: this replaces the shaft-diameter push (d_s/shaft_diam)
            % that used to live here. d_s will go stale again relative to
            % the sized rotor bore (IPMRotorSizer.RotorID_mm) -- see the
            % history this comment used to describe -- unless it's kept
            % in sync some other way (hand-set in COMSOL Desktop, or a
            % future pushGeometry option).
            obj.setParam_('mag_h', obj.lengthStr_(options.MagnetHeight_mm));
            obj.setPartInput_(obj.rotorPartTag, 'magnet_h', 'mag_h');
         end
         if isfield(options, 'StatorYokeHeight_mm')
            obj.setParam_('h_sy', obj.lengthStr_(options.StatorYokeHeight_mm));
            % backiron_th on pi2 is NOT wired to h_sy by default in this
            % model -- it's a plain literal (see COMSOL_models history:
            % it was hand-tuned through several literal mm values before
            % ever being pointed at "h_sy"). Re-pin it every push so the
            % two can't silently drift apart again.
            obj.setPartInput_(obj.statorPartTag, 'backiron_th', 'h_sy');
         end
         if isfield(options, 'L_mm')
            % L already drives the Rotating Machinery, Magnetic physics'
            % out-of-plane Thickness (d = L), set once when the model was
            % built from the Application Library recipe -- no part
            % instance rewiring needed here.
            obj.setParam_('L', obj.lengthStr_(options.L_mm));
         end

         if isfield(options, 'MaxSpeed_rpm')
            % w_rot drives the coupled structural sweep's rotation speed
            % (and, via f_el=w_rot*Np/2, the electrical frequency/time
            % step of the whole study) -- it's the worst-case speed the
            % rotor-bridge stress check should run at, so it must track
            % this design's actual max speed, not whatever the template
            % last had it set to. Not part of the original six pushed
            % values; added so it can't silently drift the way
            % backiron_th used to (see that field's comment above).
            obj.setParam_('w_rot', sprintf('%.6g[rpm]', options.MaxSpeed_rpm));
         end

         % d_cont (stator-rotor continuity interface) is already the
         % formula d_r+airgap in Global Definitions, so it tracks d_r and
         % airgap automatically. Re-pin both part instances to it anyway
         % -- cheap, and removes any doubt if it was ever hand-overridden.
         if isfield(options, 'RotorDiameter_mm') || isfield(options, 'Airgap_mm')
            obj.setPartInput_(obj.rotorPartTag,  'cont_diam', 'd_cont');
            obj.setPartInput_(obj.statorPartTag, 'cont_diam', 'd_cont');
         end
      end

      function pushExcitation(obj, options)
         %pushExcitation  Set the stator coil excitation (Ipk, init_ang
         %Global Parameters). Both already feed the rmm physics' Multiphase
         %Winding feature (wnd1) via a symbolic reference ("Ipk"/"init_ang"),
         %not a literal -- confirmed from the model's own edit history --
         %so no physics-feature rewiring is needed here, unlike backiron_th.
         %
         % init_ang is deliberately NOT part of this method's inputs: the
         % Application Library's own doc describes it as "the initial
         % current angle for peak torque" -- i.e. tuned so the current
         % vector sits on the q-axis (Id=0), which is exactly what our
         % sizing already assumes (IPMRotorSizer.m: "Id = 0; % Assuming no
         % field weakening for the base torque point"). Since both sides
         % target the same Id=0/MTPA condition, the template's existing
         % init_ang should already be correct for that alignment; only the
         % current magnitude (Ipk) needs to move with the design. If a
         % geometry change ever shifts the true peak-torque angle (e.g.
         % saturation/cross-coupling not captured by the Id=0 assumption),
         % that would need an angle sweep in COMSOL to re-tune -- out of
         % scope here.
         arguments
            obj
            options.Ipk_A (1,1) double
         end

         obj.ensureModelReady_();

         if isfield(options, 'Ipk_A')
            obj.setParam_('Ipk', sprintf('%.6g[A]', options.Ipk_A));
         end
      end

      function rebuildGeometry(obj)
         %rebuildGeometry  Re-run the geometry sequence so pushed values
         %take effect. COMSOL's own Parameter Check nodes on pi1/pi2
         %(e.g. backiron_th <= bit_u_l) will throw a clear error here if
         %a pushed combination is geometrically infeasible.
         obj.ensureModelReady_();
         fprintf('[ComsolPrebuiltInterface] Rebuilding geometry (%s)...\n', obj.geomTag);
         try
            obj.model.geom(obj.geomTag).run();
         catch ME
            error(['Geometry rebuild failed after pushing values -- the new dimensions are ' ...
                  'likely geometrically inconsistent (see COMSOL error below).\n%s'], ME.message);
         end
         fprintf('[ComsolPrebuiltInterface] Geometry rebuilt successfully.\n');
      end

      function summary(obj)
         %summary  Print the eight pushed values back as currently
         %evaluated by the model, plus the derived d_cont/d_st, as a
         %quick confirmation that the push landed correctly.
         obj.ensureModelReady_();
         m = obj.model;

         fprintf('\nComsolPrebuiltInterface summary (template: %s)\n', obj.templatePath);
         fprintf('%s\n', repmat('-', 1, 52));
         obj.printParam_(m, 'Np',     '',   'Poles (Np):');
         obj.printParam_(m, 'Ns',     '',   'Slots (Ns):');
         obj.printParam_(m, 'airgap', 'mm', 'Airgap:');
         obj.printParam_(m, 'h_sy',   'mm', 'Stator yoke height (h_sy):');
         obj.printParam_(m, 'd_r',    'mm', 'Rotor diameter (d_r):');
         obj.printParam_(m, 'mag_h',  'mm', 'Magnet height (mag_h):');
         obj.printParam_(m, 'L',      'mm', 'Stack length (L):');
         obj.printParam_(m, 'd_cont', 'mm', 'Rotor/stator interface (d_cont, derived from d_r+airgap):');
         obj.printParam_(m, 'd_st',   'mm', 'Stator diameter (d_st, derived from h_sy):');
         obj.printParam_(m, 'Ipk',    'A',  'Phase current peak (Ipk):');
         obj.printParam_(m, 'init_ang', 'deg', 'Initial current angle (init_ang):');
         obj.printParam_(m, 'w_rot',  'rpm','Rotation speed for structural sweep (w_rot):');
         obj.printParam_(m, 'N_tsteps', '', 'Time steps (N_tsteps):');
         obj.printParam_(m, 't_end',  's',  'Sweep duration (t_end):');
         fprintf('%s\n\n', repmat('-', 1, 52));
      end

      function configureSolveSettings(obj, options)
         %configureSolveSettings  Trade solve time for precision, for
         %draft/iteration runs. Leave everything at its default (i.e.
         %don't call this at all) for the numbers that go in the report.
         %
         %   MeshLevel            1 (extremely fine) .. 9 (extremely
         %                        coarse), COMSOL's own predefined mesh
         %                        size scale -- reuses
         %                        MotorMaterials.mesh_size, which
         %                        previously had nothing wired up to it.
         %   NumElectricalCycles  how many electrical periods the
         %                        Time Dependent step sweeps (template
         %                        default: 3). Fewer cycles means less
         %                        settled torque-ripple/stress statistics.
         %   TimeStepsPerCycle    time steps per electrical period
         %                        (template default: 24, i.e. 72/3).
         %                        Fewer steps means a coarser transient.
         arguments
            obj
            options.MeshLevel (1,1) double {mustBeInteger, mustBeInRange(options.MeshLevel, 1, 9)}
            options.NumElectricalCycles (1,1) double {mustBePositive}
            options.TimeStepsPerCycle (1,1) double {mustBePositive, mustBeInteger}
         end
         obj.ensureModelReady_();

         if isfield(options, 'MeshLevel')
            try
               obj.model.mesh(obj.meshTag).feature('size').set('hauto', num2str(options.MeshLevel));
               fprintf('[ComsolPrebuiltInterface] Mesh level set to %d (1=finest, 9=coarsest); re-run rebuildGeometry()/mesh to apply.\n', ...
                       options.MeshLevel);
            catch ME
               error('Failed setting mesh level on "%s".feature(''size''). \nOriginal error: %s', obj.meshTag, ME.message);
            end
         end

         if isfield(options, 'NumElectricalCycles') || isfield(options, 'TimeStepsPerCycle')
            % Both are needed together since N_tsteps (total steps) is
            % expressed in the template relative to a whole number of
            % cycles; read back whichever wasn't given so one can be
            % changed without silently resetting the other.
            if isfield(options, 'NumElectricalCycles')
               nCycles = options.NumElectricalCycles;
            else
               nCycles = obj.model.param.evaluate('t_end') / (1/obj.model.param.evaluate('f_el'));
            end
            if isfield(options, 'TimeStepsPerCycle')
               stepsPerCycle = options.TimeStepsPerCycle;
            else
               stepsPerCycle = obj.model.param.evaluate('N_tsteps') / nCycles;
            end
            obj.setParam_('t_end', sprintf('%.6g/f_el', nCycles));
            obj.setParam_('N_tsteps', sprintf('%d', round(nCycles*stepsPerCycle)));
            fprintf('[ComsolPrebuiltInterface] Sweep set to %.3g electrical cycle(s), %d step(s)/cycle.\n', ...
                    nCycles, round(stepsPerCycle));
         end
      end

      function saveModelAs(obj, savePath)
         %saveModelAs  Save the pushed/rebuilt model. Defaults to
         %OutputPath if no path is given. Refuses to write to
         %TemplatePath -- even if passed explicitly -- so the clean
         %template can never be clobbered by this class.
         arguments
            obj
            savePath (1,:) char = ''
         end
         if isempty(savePath)
            savePath = obj.outputPath;
         end
         if ComsolPrebuiltInterface.samePath_(savePath, obj.templatePath)
            error(['Refusing to save to TemplatePath (%s) -- it is meant to stay a clean, ' ...
                  'untouched baseline. Pass a different path, or use the default OutputPath ' ...
                  '(%s) by calling saveModelAs() with no argument.'], obj.templatePath, obj.outputPath);
         end

         obj.ensureModelReady_();
         saveDir = fileparts(savePath);
         if ~isempty(saveDir) && ~exist(saveDir, 'dir')
            mkdir(saveDir);
         end
         obj.model.save(savePath);
         fprintf('[ComsolPrebuiltInterface] Model saved to: %s\n', savePath);
      end

      function runStudy(obj)
         %runStudy  Run Study 1 (std1): a Stationary step followed by a
         %Time Dependent sweep coupling Rotating Machinery, Magnetic
         %(rmm) with Solid Mechanics as the rotor turns through several
         %electrical cycles (see the model's w_rot/N_tsteps/t_end
         %parameters). This is a real transient FEA solve -- expect it
         %to take minutes, not seconds.
         obj.ensureModelReady_();
         fprintf('[ComsolPrebuiltInterface] Running study std1 (stationary + time-dependent sweep)...\n');
         try
            obj.model.study('std1').run();
         catch ME
            error('Study run failed.\n%s', ME.message);
         end
         fprintf('[ComsolPrebuiltInterface] Study completed.\n');
      end

      function [s, fromCache] = runStudyCached(obj, options)
         %runStudyCached  Run the study, or reuse a previous run's
         %results, keyed off the design-point values currently pushed
         %into the model (Np, Ns, airgap, h_sy, d_r, L, w_rot, Ipk) --
         %not off wall-clock time or the .mph file's contents. Re-solving
         %this study is the slow part of the whole pipeline (minutes, not
         %seconds); this lets main.m be re-run repeatedly (e.g. while
         %iterating on the report) without waiting on COMSOL again unless
         %the design actually changed.
         %
         %   [s, fromCache] = ci.runStudyCached()
         %   [s, fromCache] = ci.runStudyCached(Force=true)   % re-solve regardless
         %   [s, fromCache] = ci.runStudyCached(FigureDir=...) % also (re-)render plots on a fresh solve
         %
         % Cache files live in COMSOL_models/cache/ (gitignored, like the
         % .mph outputs -- regenerable, not meant to be committed) as
         % <design-point-values>.mat, so the filename alone tells you
         % which design point a cached result belongs to.
         arguments
            obj
            options.Force (1,1) logical = false
            options.CacheDir (1,:) char = ''
            options.FigureDir (1,:) char = ''
         end
         obj.ensureModelReady_();

         if isempty(options.CacheDir)
            options.CacheDir = fullfile(ComsolPrebuiltInterface.defaultProjectRoot(), 'COMSOL_models', 'cache');
         end
         if ~exist(options.CacheDir, 'dir')
            mkdir(options.CacheDir);
         end
         cacheFile = fullfile(options.CacheDir, [obj.designFingerprint_() '.mat']);

         if ~options.Force && exist(cacheFile, 'file') == 2
            loaded = load(cacheFile, 's', 'timestamp');
            s = loaded.s;
            fromCache = true;
            fprintf('[ComsolPrebuiltInterface] Reusing cached study results from %s\n', loaded.timestamp);
            fprintf('  (design point unchanged since that run -- pass Force=true to re-solve anyway)\n');
            obj.printResults_(s);
            return;
         end

         obj.runStudy();
         s = obj.resultsSummary();
         fromCache = false;

         timestamp = char(datetime('now'));
         save(cacheFile, 's', 'timestamp');
         fprintf('[ComsolPrebuiltInterface] Cached results to %s\n', cacheFile);

         if ~isempty(options.FigureDir)
            obj.plotAll(options.FigureDir);
         end
      end

      function s = resultsSummary(obj)
         %resultsSummary  Pull the key scalar results out of the solved
         %model and print them: axial torque (rmm.Tark_1, the Arkkio
         %method torque built into the Rotating Machinery, Magnetic
         %physics) averaged and rippled over the time sweep, plus peak
         %von Mises stress and peak displacement from the coupled Solid
         %Mechanics solve. Returns everything in a struct too.
         obj.ensureModelReady_();
         s = struct();

         try
            s.torque_Nm = mphglobal(obj.model, 'rmm.Tark_1');
            s.torqueAvg_Nm = mean(s.torque_Nm);
            s.torqueRipple_pct = (max(s.torque_Nm) - min(s.torque_Nm)) / s.torqueAvg_Nm * 100;
         catch ME
            warning('Could not extract torque (rmm.Tark_1): %s', ME.message);
         end
         try
            % 'surface' = element dimension to evaluate over (this is a
            % 2D model); mphmax returns one max per solved time step, so
            % take the overall peak across the whole sweep. solid.misesGp
            % is a stress field, always native in Pa regardless of the
            % model's mm length unit -- request MPa explicitly.
            misesPerStep = mphmax(obj.model, 'solid.misesGp', 'surface', 'unit', 'MPa');
            s.maxVonMises_MPa = max(misesPerStep(:));
         catch ME
            warning('Could not extract max von Mises stress (solid.misesGp): %s', ME.message);
         end
         try
            % solid.disp is a length quantity, so it already comes out in
            % the model's own length unit (mm) with no unit override needed.
            dispPerStep = mphmax(obj.model, 'solid.disp', 'surface');
            s.maxDisplacement_mm = max(dispPerStep(:));
         catch ME
            warning('Could not extract max displacement (solid.disp): %s', ME.message);
         end

         obj.printResults_(s);
      end

      function fig = plot(obj, plotGroupTag, options)
         %plot  Render one of the model's own pre-built Result plot
         %groups (pg1..pg8, see below) via mphplot, rather than
         %reassembling plot settings from scratch. Optionally saves a
         %PNG. Returns the figure handle.
         %
         % Plot groups already defined in the Application Library model:
         %   pg1  Von Mises stress (surface)
         %   pg2  Von Mises stress (line, along a path)
         %   pg3  Magnetic flux density norm + field lines (surface+streamline+contour)
         %   pg4  Mesh
         %   pg5  Stress tensor, local 11-component (surface)
         %   pg6  Displacement magnitude (surface)
         %   pg7  Bridge point displacement, X/Y components vs. time (1D)
         %   pg8  Axial torque (rmm.Tark_1) vs. time (1D)
         arguments
            obj
            plotGroupTag (1,:) char
            options.SavePath (1,:) char = ''
            options.Title (1,:) char = ''
         end
         obj.ensureModelReady_();

         fig = figure('Name', sprintf('ComsolPrebuiltInterface: %s', plotGroupTag));
         try
            mphplot(obj.model, plotGroupTag, 'rangenum', 1);
            % The 1D time-series groups (pg7/pg8) can carry a stale fixed
            % x-axis range from whatever t_end the template was last
            % saved with in COMSOL Desktop -- harmless to also apply to
            % the spatial surface plots (their axes already match the
            % geometry extents, so 'auto' is a no-op there).
            xlim(gca, 'auto'); ylim(gca, 'auto');
         catch ME
            close(fig);
            error('Failed to render plot group "%s". Confirm it still exists in Results.\n%s', ...
                  plotGroupTag, ME.message);
         end
         if ~isempty(options.Title)
            title(options.Title, 'Interpreter', 'none');
         end
         if ~isempty(options.SavePath)
            saveDir = fileparts(options.SavePath);
            if ~isempty(saveDir) && ~exist(saveDir, 'dir')
               mkdir(saveDir);
            end
            saveas(fig, options.SavePath);
            fprintf('[ComsolPrebuiltInterface] Plot saved to: %s\n', options.SavePath);
         end
      end

      function fig = plotTorque(obj, options)
         arguments
            obj
            options.SavePath (1,:) char = ''
         end
         fig = obj.plot('pg8', SavePath=options.SavePath, Title='Axial torque (rmm.Tark_1) vs. time');
      end

      function fig = plotStress(obj, options)
         arguments
            obj
            options.SavePath (1,:) char = ''
         end
         fig = obj.plot('pg1', SavePath=options.SavePath, Title='Von Mises stress');
      end

      function fig = plotFluxDensity(obj, options)
         arguments
            obj
            options.SavePath (1,:) char = ''
         end
         fig = obj.plot('pg3', SavePath=options.SavePath, Title='Magnetic flux density norm');
      end

      function fig = plotDisplacement(obj, options)
         arguments
            obj
            options.SavePath (1,:) char = ''
         end
         fig = obj.plot('pg6', SavePath=options.SavePath, Title='Displacement magnitude');
      end

      function figs = plotAll(obj, outputDir)
         %plotAll  Convenience one-shot: render+save torque, stress,
         %flux density, and displacement together. Pass outputDir to
         %save PNGs (e.g. one per main.m run); omit to just pop up the
         %figures.
         arguments
            obj
            outputDir (1,:) char = ''
         end
         savePathFor = @(name) '';
         if ~isempty(outputDir)
            if ~exist(outputDir, 'dir')
               mkdir(outputDir);
            end
            savePathFor = @(name) fullfile(outputDir, [name '.png']);
         end

         figs = struct();
         figs.torque       = obj.plotTorque(SavePath=savePathFor('torque'));
         figs.stress        = obj.plotStress(SavePath=savePathFor('von_mises_stress'));
         figs.fluxDensity   = obj.plotFluxDensity(SavePath=savePathFor('flux_density'));
         figs.displacement  = obj.plotDisplacement(SavePath=savePathFor('displacement'));
      end
   end

   methods (Access = private)
      function ensureModelReady_(obj)
         if ~isempty(obj.model)
            return;
         end
         if exist(obj.templatePath, 'file') ~= 2
            error('Template model file not found: %s', obj.templatePath);
         end

         ComsolPrebuiltInterface.ensureComsolConnected_();

         fprintf('[ComsolPrebuiltInterface] Loading template: %s...\n', obj.templatePath);
         obj.model = mphload(obj.templatePath);
         fprintf('[ComsolPrebuiltInterface] Model loaded (headless copy; not linked to any open Desktop session).\n');
      end

      function setParam_(obj, name, expr)
         try
            obj.model.param.set(name, expr);
         catch ME
            error(['Failed setting Global Parameter "%s" to "%s". Confirm the parameter exists ' ...
                  'in this model (Global Definitions > Parameters).\nOriginal error: %s'], ...
                  name, expr, ME.message);
         end
      end

      function setPartInput_(obj, partTag, inputName, expr)
         try
            obj.model.geom(obj.geomTag).feature(partTag).setEntry('inputexpr', inputName, expr);
         catch ME
            error(['Failed setting Part Instance "%s" input "%s" to "%s". Confirm the part ' ...
                  'instance tag and input parameter name still match the model ' ...
                  '(Geometry > %s > Settings > Input Parameters).\nOriginal error: %s'], ...
                  partTag, inputName, expr, partTag, ME.message);
         end
      end

      function str = lengthStr_(~, value_mm)
         str = sprintf('%.6g[mm]', value_mm);
      end

      function printParam_(~, model, name, unit, label)
         % Print both the raw expression (what's actually stored, e.g.
         % "d_r+airgap") and its numerically evaluated value, so a
         % symbolic reference and a hardcoded literal are easy to tell
         % apart at a glance.
         try
            expr = char(model.param.get(name));
         catch
            fprintf('  %-30s (parameter not found)\n', label);
            return;
         end
         try
            if isempty(unit)
               value = model.param.evaluate(name);
               valueStr = sprintf('%.6g', value);
            else
               value = model.param.evaluate(name, unit);
               valueStr = sprintf('%.6g %s', value, unit);
            end
         catch
            valueStr = '(could not evaluate)';
         end
         fprintf('  %-30s %-12s [%s]\n', label, valueStr, expr);
      end

      function printResults_(~, s)
         % Shared by resultsSummary() (fresh solve) and runStudyCached()
         % (cache hit) so both paths print identically.
         fprintf('\nComsolPrebuiltInterface results summary\n');
         fprintf('%s\n', repmat('-', 1, 52));
         if isfield(s, 'torqueAvg_Nm')
            fprintf('  %-30s %.3f Nm\n', 'Average axial torque:', s.torqueAvg_Nm);
            fprintf('  %-30s %.2f %%\n', 'Torque ripple:', s.torqueRipple_pct);
         end
         if isfield(s, 'maxVonMises_MPa')
            fprintf('  %-30s %.2f MPa\n', 'Peak von Mises stress:', s.maxVonMises_MPa);
         end
         if isfield(s, 'maxDisplacement_mm')
            fprintf('  %-30s %.4f mm\n', 'Peak displacement:', s.maxDisplacement_mm);
         end
         fprintf('%s\n\n', repmat('-', 1, 52));
      end

      function v = designFingerprintValues_(obj)
         m = obj.model;
         v.Np         = m.param.evaluate('Np');
         v.Ns         = m.param.evaluate('Ns');
         v.airgap_mm  = m.param.evaluate('airgap', 'mm');
         v.h_sy_mm    = m.param.evaluate('h_sy',   'mm');
         v.d_r_mm     = m.param.evaluate('d_r',    'mm');
         v.L_mm       = m.param.evaluate('L',      'mm');
         v.w_rot_rpm  = m.param.evaluate('w_rot',  'rpm');
         v.Ipk_A      = m.param.evaluate('Ipk',    'A');
         v.N_tsteps   = m.param.evaluate('N_tsteps');
      end

      function fp = designFingerprint_(obj)
         % A human-readable cache key built directly from the currently
         % pushed design values -- no hashing needed, and the filename
         % alone tells you which design point a cache entry belongs to.
         v = obj.designFingerprintValues_();
         fp = sprintf('Np%g_Ns%g_ag%.4f_hsy%.4f_dr%.4f_L%.4f_wrot%.1f_Ipk%.3f_nt%g', ...
                       v.Np, v.Ns, v.airgap_mm, v.h_sy_mm, v.d_r_mm, v.L_mm, ...
                       v.w_rot_rpm, v.Ipk_A, v.N_tsteps);
      end
   end

   methods (Static, Access = private)
      function projectRoot = defaultProjectRoot()
         classFile = which('ComsolPrebuiltInterface');
         if isempty(classFile)
            projectRoot = pwd;
            return;
         end
         projectRoot = fullfile(fileparts(classFile), '..');
      end

      function ensureComsolConnected_()
         % Add the LiveLink for MATLAB toolbox to the path (if it isn't
         % already) and connect to a running "comsol mphserver". Checks
         % COMSOL_MLI_DIR first, then this machine's known install
         % location; set COMSOL_MLI_DIR if yours lives somewhere else.
         if isempty(which('mphstart'))
            comsolMliDir = getenv('COMSOL_MLI_DIR');
            if isempty(comsolMliDir)
               candidates = {
                  '/opt/comsol64/multiphysics/mli'
                  fullfile(getenv('HOME'), 'opt', 'comsol64', 'multiphysics', 'mli')
                  '/usr/local/comsol64/multiphysics/mli'
               };
               for i = 1:numel(candidates)
                  if exist(candidates{i}, 'dir') == 7
                     comsolMliDir = candidates{i};
                     break;
                  end
               end
            end
            if isempty(comsolMliDir) || exist(comsolMliDir, 'dir') ~= 7
               error(['Could not find the COMSOL LiveLink for MATLAB toolbox (mphstart, mphload, ...). ' ...
                     'Set the COMSOL_MLI_DIR environment variable to its "mli" folder.']);
            end
            addpath(comsolMliDir);
         end

         try
            fprintf('[ComsolPrebuiltInterface] Checking for COMSOL Server connection...\n');
            mphstart;
            fprintf('[ComsolPrebuiltInterface] Successfully connected to COMSOL Server.\n');
         catch ME
            if contains(ME.message, 'Already connected')
               fprintf('[ComsolPrebuiltInterface] COMSOL connection already active.\n');
            elseif contains(ME.message, 'Connection refused') || contains(ME.message, 'ConnectException')
               error('[COMSOL ERROR]: Server not found. Run "comsol mphserver" in a terminal first.');
            else
               error('Unexpected COMSOL connection error: %s', ME.message);
            end
         end
      end

      function tf = samePath_(pathA, pathB)
         % Compare two paths by canonical (absolute, resolved) form, so
         % "./COMSOL_models/x.mph" and an absolute path to the same file
         % are correctly recognized as identical.
         canon = @(p) char(java.io.File(p).getCanonicalPath());
         try
            tf = strcmp(canon(pathA), canon(pathB));
         catch
            % One of the paths doesn't exist yet (e.g. output file not
            % created yet) -- fall back to a plain string comparison.
            tf = strcmp(pathA, pathB);
         end
      end
   end
end
