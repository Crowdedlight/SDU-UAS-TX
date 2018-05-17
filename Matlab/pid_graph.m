clear; clc;
path = "../remotecontrol_ws/src/camera_controller/ErrorLogs/";

file = csvread(path + "finer?.csv");

%rename file after loading it
% Get all text files in the current folder
% files = dir(strcat(path + "*.csv"));
% % sort based on name
% [~, reindex] = sort( str2double( regexp( {files.name}, '\d+', 'match', 'once' )));
%     files = files(reindex);
% % Loop through each file 
% for fil = files'
%     % Get the file name 
%     if fil.name == "error.csv"
%        moveFile = fil;
%     end
%     lastName = fil.name;
% end
% 
% %get next number in sequence, if none start from 1
% N = regexp(lastName,'\d*','Match');
% num = "1";
% if ~isempty(N)
%     num = str2num(N{1});
%     num = num + 1;
% end
%     
% % move file
% movefile(path + "error.csv",path + "error" + num + ".csv");


time_start = file(1,5);
time = file(:,5)-time_start;

f = figure;
subplot(2,2,1)       % add first plot in 2 x 1 grid
plot(time, file(:,1));
title('Roll')

subplot(2,2,2)       % add second plot in 2 x 1 grid
plot(time, file(:,2));      % plot using + markers
title('Pitch')

subplot(2,2,3)       % add second plot in 2 x 1 grid
plot(time, file(:,3));      % plot using + markers
title('Thrust')

subplot(2,2,4)       % add second plot in 2 x 1 grid
plot(time, file(:,4));      % plot using + markers
title('Yaw')

% Set Axis
ax = findobj(f,'Type','Axes');
for i=1:length(ax)
    ylabel(ax(i),{'Error in cm'})
    xlabel(ax(i),{'seconds'})
end
