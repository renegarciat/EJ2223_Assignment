function results = run_all_tests()
%RUN_ALL_TESTS Discover and run every MATLAB_Code/tests/test_*.m file.
%
%   results = run_all_tests()
%
%   Locates its own directory via mfilename so it works regardless of the
%   current MATLAB folder, puts MATLAB_Code on the path, then runs each
%   test_*.m file in an isolated workspace so one
%   failure doesn't stop the rest. Prints a PASS/FAIL report and returns
%   it as a struct array (Name, Passed, Message, Elapsed_s).
%
%   Usage
%   -----
%     From MATLAB_Code/tests:              >> run_all_tests
%     From anywhere, absolute path:        >> run('/abs/path/to/MATLAB_Code/tests/run_all_tests.m')
%     From repo root, call-by-name:        >> addpath('MATLAB_Code/tests'); run_all_tests
%     (MATLAB's run() builtin can mis-resolve multi-level *relative*
%     paths like run('MATLAB_Code/tests/run_all_tests.m') from outside
%     the tests folder -- use one of the forms above instead.)
%     >> results = run_all_tests();  % also returns a struct array

    thisDir       = fileparts(mfilename('fullpath'));  % .../MATLAB_Code/tests
    matlabCodeDir = fileparts(thisDir);                 % .../MATLAB_Code
    addpath(matlabCodeDir);
    addpath(thisDir);

    testFiles = dir(fullfile(thisDir, 'test_*.m'));
    testFiles = testFiles(~[testFiles.isdir]);
    n = numel(testFiles);

    results = struct('Name', {}, 'Passed', {}, 'Message', {}, 'Elapsed_s', {});

    fprintf('\nRunning %d test file(s) from %s\n', n, thisDir);
    fprintf('%s\n', repmat('=', 1, 70));

    for i = 1:n
        name     = testFiles(i).name;
        testPath = fullfile(thisDir, name);
        fprintf('\n--- %s ---\n', name);

        t0 = tic;
        [passed, message] = runSingleTest_(testPath);
        elapsed = toc(t0);

        results(end+1) = struct('Name', name, 'Passed', passed, ...
            'Message', message, 'Elapsed_s', elapsed); %#ok<AGROW>

        if passed
            fprintf('[PASS] %s (%.2fs)\n', name, elapsed);
        else
            fprintf('[FAIL] %s (%.2fs): %s\n', name, elapsed, message);
        end
    end

    fprintf('\n%s\n', repmat('=', 1, 70));
    fprintf('Results report\n');
    fprintf('%s\n', repmat('-', 1, 70));
    for i = 1:n
        status = 'FAIL';
        if results(i).Passed
            status = 'PASS';
        end
        fprintf('  [%s] %-35s %6.2fs\n', status, results(i).Name, results(i).Elapsed_s);
    end
    fprintf('%s\n', repmat('-', 1, 70));

    nPassed = sum([results.Passed]);
    fprintf('%d/%d test files passed\n', nPassed, n);
    fprintf('%s\n\n', repmat('=', 1, 70));

    if nPassed < n
        fprintf(2, '%d test file(s) FAILED.\n', n - nPassed);
    else
        fprintf('All tests passed.\n');
    end
end

function [passed, message] = runSingleTest_(testPath)
    % Runs in its own function workspace so a test's "clear" doesn't wipe
    % out run_all_tests' loop state (i, n, results, ...).
    try
        run(testPath);
        passed  = true;
        message = '';
    catch ME
        passed  = false;
        message = ME.message;
    end
end
