
% Get the full path of the current running script
currentScriptPath = mfilename('fullpath');

% Extract the folder containing the script
[currentScriptFolder, ~, ~] = fileparts(currentScriptPath);

% Change the current directory to that folder
cd(currentScriptFolder);
