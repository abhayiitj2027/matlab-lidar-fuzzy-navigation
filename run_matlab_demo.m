function result = run_matlab_demo(makeVideo)
% RUN_MATLAB_DEMO Native MATLAB LiDAR / local SLAM / fuzzy navigation.
% No Python bridge, payload model, MPC, or additional toolbox is required.
% Run from this project folder: result = run_matlab_demo;
if nargin<1, makeVideo=true; end
root=fileparts(mfilename('fullpath'));
out=fullfile(root,'output'); if ~exist(out,'dir'), mkdir(out); end
result=simulate_lidar;
save(fullfile(out,'simulation_result.mat'),'result','-v7.3');
writetable(array2table(result.telemetry,'VariableNames', ...
    {'time','true_x','true_y','estimated_x','estimated_y','speed','turn','clearance','replans'}), ...
    fullfile(out,'telemetry.csv'));
fid=fopen(fullfile(out,'summary.json'),'w');
assert(fid>=0,'Cannot write summary');
fprintf(fid,'%s',jsonencode(result.summary)); fclose(fid);
disp(result.summary);
if makeVideo
    result.video=render_lidar_video(result,out);
    fprintf('MATLAB generated video: %s\n',result.video);
end
end
