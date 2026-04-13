function results = run_comsol_simulation(params, config)
%RUN_COMSOL_SIMULATION Run the COMSOL FEA workflow from MATLAB.
%
% This file is a thin wrapper around the class-based implementation in
% ComsolInterface.
%
% Usage:
%   results = run_comsol_simulation();
%   results = run_comsol_simulation(params);
%   results = run_comsol_simulation(params, config);
%
% params: struct of COMSOL global parameters (name -> expression string).
% config: struct with optional fields:
%   - comsolMliDir
%   - modelPath
%   - studyTag
%   - resultTableTag
%   - saveResults (true/false)
%   - savePath
%   - makePlot (true/false)

    if nargin < 1
        params = struct();
    end
    if nargin < 2
        config = struct();
    end

    results = ComsolInterface.runComsolSimulation(params, config);
end
