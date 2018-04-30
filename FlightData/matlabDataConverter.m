close all; 

%% ------------------------------------------------------------------------
% File creation
% ------------------------------------------------------------------------

name = 'SimDataPayerne.h';

delete(name);

% open file
headerId = fopen(name, 'w');

%% ------------------------------------------------------------------------
% Table definiton
% ------------------------------------------------------------------------
wait_time = 10;
up_time = 0.1;
T_wait = (0:up_time:wait_time)';

T = [T_wait;T1+wait_time;T2(2:end)+wait_time; T3(2:end)+wait_time];
Altitude = [zeros(length(T_wait), 1); X1(:,1); X2(2:end,3); X3(2:end,3)];
Velocity = [zeros(length(T_wait), 1); X1(:,2); X2(2:end,4); X3(2:end,4)];
Pressure = [];
for i = 1:length(T)
    [~,~,p,~] = stdAtmos(Altitude(i));    
    Pressure = [Pressure; p];
end
Acceleration = diff(Velocity)./diff(T)/9.81;
Acceleration(isnan(Acceleration)) = 0;
Acceleration = [Acceleration; Acceleration(end)];
data_table = [T, Altitude, Pressure, Acceleration];

%% ------------------------------------------------------------------------
% Header writing
% ------------------------------------------------------------------------

% write header data
fprintf(headerId, '#ifndef INCLUDE_SIM_DATA_ \n');
fprintf(headerId, '#define INCLUDE_SIM_DATA_ \n\n');
fprintf(headerId, ['#define SIM_TAB_HEIGHT ' num2str(size(data_table,1)) '\n']);
fprintf(headerId, ['#define SIM_TAB_WIDTH ' num2str(size(data_table,2)) '\n']);
fprintf(headerId, '#define SIM_TIMESTAMP 0\n');
fprintf(headerId, '#define SIM_ALTITUDE 1\n');
fprintf(headerId, '#define SIM_PRESSURE 2\n');
fprintf(headerId, '#define SIM_ACCELX 3\n');
fprintf(headerId, ['static const float32_t SimData[' num2str(size(data_table,1)) '][' num2str(size(data_table,2)) '] = {\n']);

% populate Array
for i = 1:size(data_table,1)
   fprintf(headerId, [writeCArray(data_table(i,:)) ',\n']); 
end

fprintf(headerId, '};\n');
fprintf(headerId, '#endif');

fclose(headerId);

%% ------------------------------------------------------------------------
% Plot Data
% ------------------------------------------------------------------------

figure; hold on;
title 'Altitude'
plot(T, Altitude);

figure; hold on;
title 'Acceleration'
plot(T, Acceleration);