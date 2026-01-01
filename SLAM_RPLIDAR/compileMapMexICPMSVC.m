function compileMapMexICPMSVC()
    % Base source files
    sourceFiles = {'MapICPMex.cpp'};
   
    % Root paths (relative to current folder)
    eigenInclude   = fullfile(pwd, 'eigen-3.4.0');
    octomapInclude = fullfile(pwd, 'octomap', 'include');
    octomapSrc     = fullfile(pwd, 'octomap', 'src');

    % --- Collect OctoMap .cpp sources recursively ---
    cppFiles = dir(fullfile(octomapSrc, '**', '*.cpp'));
    octomapSources = fullfile({cppFiles.folder}, {cppFiles.name});
    sourceFiles = [sourceFiles, octomapSources];

    % --- Collect include paths (recursive) ---
    includeDirs = strsplit(genpath(octomapInclude), pathsep);
    includeDirs = includeDirs(~cellfun('isempty', includeDirs));

    % Add Eigen and current folder (for kdtree.h)
    includeDirs = [{eigenInclude}, includeDirs, {pwd}];

    % Normalize to forward slashes
    includeDirs = cellfun(@(d) strrep(d, '\', '/'), includeDirs, 'UniformOutput', false);

    % Build include flags (quoted)
    includeFlags = strjoin(cellfun(@(d) ['-I"', d, '"'], includeDirs, 'UniformOutput', false));

    % --- MSVC compiler flags ---
    compFlags = '/std:c++20 /O2 /fp:fast /DNDEBUG';

    % Convert source files to string (quoted)
    sourceStr = strjoin(cellfun(@(s) ['"', strrep(s, '\', '/'), '"'], sourceFiles, 'UniformOutput', false));

    % Construct mex command for MSVC
    mexCommand = sprintf('mex %s COMPFLAGS="$COMPFLAGS %s" %s', ...
        sourceStr, compFlags, includeFlags);

    fprintf('Running: %s\n', mexCommand);
    eval(mexCommand);

    fprintf('Compilation complete.\n');
end