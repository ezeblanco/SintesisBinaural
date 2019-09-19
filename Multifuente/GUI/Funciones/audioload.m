function [nombre, directorio] = audioload(hrtf_Fs)
%
[nombre_audio,directorio] = uigetfile('*.*','Elija el archivo de audio');
oldFolder = cd(directorio);
[audio,Fs] = audioread(strcat(directorio,nombre_audio));
formato = strsplit(nombre_audio,'.');
formato = strlength(formato(length(formato)))+1;
cd(oldFolder)
cd('Temp')

if Fs ~= hrtf_Fs % si la Fs del audio no coincide con los filtros
    [P,Q] = rat(hrtf_Fs/Fs);
    audio = resample(audio * 0.9,P,Q); % 0.9 constante para que no clipee cuando convierta
    if size(audio,2)==1 % si es mono
    audio = [audio audio]; % hago dos canales
    end
    nombre_audio = [nombre_audio(1,1:length(nombre_audio)-formato) '_' num2str(hrtf_Fs) '.wav'];
    audiowrite(nombre_audio,audio,hrtf_Fs);
    directorio = [oldFolder '\Temp\'];
    
elseif size(audio,2)==1 % si es mono
    audio = [audio audio]; % hago dos canales
    nombre_audio = [nombre_audio(1,1:length(nombre_audio)-formato) '_2.wav']; % num2str(handles.hrtf_Fs)
    audiowrite(nombre_audio,audio,hrtf_Fs);
    directorio = [oldFolder '\Temp\'];
end
nombre = nombre_audio;
cd(oldFolder)
end

