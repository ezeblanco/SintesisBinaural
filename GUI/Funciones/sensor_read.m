function [yaw,pitch] = sensor_read(sensor)
% Se obtiene la información del sensor
% flushinput(sensor);
[~] = strsplit(fscanf(sensor),','); % Se ignora la primera lectura
data = strsplit(fscanf(sensor),','); % Relectura para evitar errores
yaw = str2double(data(1));
pitch = str2double(data(2));
end

