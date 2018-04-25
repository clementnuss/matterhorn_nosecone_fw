% DATACONVERTER
%
% This script converts flight data into a c formated 2D array.
% The array is composed of the epooch, the xyz acceleration values and the
% barometer data.

% ------------------------------------------------------------------------
% Initialization
% ------------------------------------------------------------------------

clear all; close all;
dataFolderPath = 'EricFlight';

% ------------------------------------------------------------------------
% Data retreeval
% ------------------------------------------------------------------------

% get all files in directory 
files = dir(dataFolderPath);
% remove '.' and '..' paths
files = files(~(strcmp({files.name},'.') | strcmp({files.name},'..')));
nfiles = length(files);

filesName = {files.name};
dataArray = [];
for i = 1:nfiles
   dataArray = [dataArray; importDataFile([dataFolderPath '/' filesName{i}])];   
   display(['imported ' filesName{i}]);
end

%% ------------------------------------------------------------------------
% Header writing
% ------------------------------------------------------------------------
delete('SimData.h');

% open file
headerId = fopen('SimData.h', 'w');
% write header data
fprintf(headerId, ['#define SIM_TAB_HEIGHT ' num2str(height(dataArray)) '\n']);
fprintf(headerId, ['#define SIM_TAB_WIDTH ' num2str(3) '\n']);
fprintf(headerId, '#define SIM_TIMESTAMP 0\n');
fprintf(headerId, '#define SIM_ALTITUDE 1\n');
fprintf(headerId, '#define SIM_ACCELX 2\n');
fprintf(headerId, ['const float32_t SimData[' num2str(height(dataArray)) '][' num2str(3) '] = {\n']);

% populate Array
for i = 1:height(dataArray)
   fprintf(headerId, [writeCArray([dataArray.timestamp(i) dataArray.altitude(i) dataArray.accelx(i)]) ',\n']); 
end

fprintf(headerId, '};\n');

fclose(headerId);