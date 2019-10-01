% Lectura de datos del magnetómetro

clear
clc
data=zeros(400,3);
B = Bluetooth('SENSOR',1);
fopen(B);

contador = 1;
while contador<400
    data(contador,:) = str2double(strsplit(fscanf(B),','));
    data(contador,:) = str2double(strsplit(fscanf(B),','));
    
    contador=contador+1;
end
fclose(B);

plot(data(:,1),data(:,2)) % Mxy
hold on
plot(data(:,1),data(:,3)) % Mxz
plot(data(:,2),data(:,3)) % Myz

grid on