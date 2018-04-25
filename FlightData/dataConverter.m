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
fprintf(headerId, 'float32_t SimData = {\n');

% populate Array
for i = 1:height(dataArray)
   fprintf(headerId, [writeCArray([dataArray.timestamp(i) dataArray.altitude(i) dataArray.accelx(i)]) ',\n']); 
end

fprintf(headerId, '};\n');

fclose(headerId);