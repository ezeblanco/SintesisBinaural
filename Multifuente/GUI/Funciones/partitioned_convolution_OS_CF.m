function [out,pad_block] = partitioned_convolution_OS_CF(audio_in,filtros,pad_block,filtros_previos)
% Convolución por bloques en el dominio de las frecuencias
% Entrada:
%   audio_in        - Buffer de audio.
%   filtro          - Par de respuestas al impulso a convolucionar.
%   pad_block       - Bloque de audio previo para rellenar. La primera vez
%                     son ceros.
%   filtros_previos - Son los filtros con la posición de fuente previa para
%                     realizar el crossfading.
% Salida:
%   out             - Buffer de salida procesado.
%   pad_block       - Bloque excedente de la convolucion para agregar en el
%                     proximo procesamiento (save and reuse).
%% Argumentos
if nargin < 4
    crossfade = 0; % no se activa el crossfade
else
    crossfade = 1;
end
%% Parametros
L1 = length(audio_in); % longitud de la señal de audio
N1 = length(filtros); % longitud de los filtros
%% Ajuste 
filtros = [filtros;zeros(N1,2)]; % pad de ceros al filtro
%% Convolución overlap-save
out = zeros(L1,2); 
fft_filtro = fft(filtros);

ii=0;
iiend=L1;

if crossfade == 1
    filtros_previos = [filtros_previos;zeros(N1,2)];
    fft_filtro_previo = fft(filtros_previos); 
    % se generan las funciones de fundido
    long_cf = 32; % Longitud del fundido cruzado N1
    fade_out = (0.5+(0.5*cos(pi*linspace(1,2*long_cf,long_cf)/(2*long_cf))))'; 
    fade_in = (0.5-(0.5*cos(pi*linspace(1,2*long_cf,long_cf)/(2*long_cf))))';
    
    while ii<iiend
        segT = [pad_block;audio_in(ii+1:ii+N1,:)]; % segmento en el dominio temporal
		segF = fft(segT);
        segFO = fft_filtro .* segF; % Multiplicación en el espectro
        segTO = real(ifft(segFO)); % Dominio temporal
        
        if ii == 0 
        segFO2 = fft_filtro_previo .* segF;
        segTO2 = real(ifft(segFO2));
        % crossfading
        if long_cf == N1
        out(ii+1:ii+N1,1) = fade_in .* segTO(N1+1:2*N1,1) + fade_out .* segTO2(N1+1:2*N1,1);
        out(ii+1:ii+N1,2) = fade_in .* segTO(N1+1:2*N1,2) + fade_out .* segTO2(N1+1:2*N1,2);
        pad_block = audio_in(ii+1:ii+N1,:);
        ii = ii+N1; 
        else % si se utiliza una longitud menor a N1
        out(ii+1:ii+long_cf,1) = fade_in .* segTO(N1+1:N1+long_cf,1) + fade_out .* segTO2(N1+1:N1+long_cf,1);
        out(ii+1:ii+long_cf,2) = fade_in .* segTO(N1+1:N1+long_cf,2) + fade_out .* segTO2(N1+1:N1+long_cf,2);
        out(ii+long_cf+1:ii+N1,:) = segTO(N1+long_cf+1:2*N1,:);
        pad_block = audio_in(ii+1:ii+N1,:);
        ii = ii+N1; 
        end
        else
        out(ii+1:ii+N1,:) = segTO(N1+1:2*N1,:);
        pad_block = audio_in(ii+1:ii+N1,:);
        ii = ii+N1; 
        end
    end   
else % crossfade == 0
    while ii<iiend
        segT = [pad_block;audio_in(ii+1:ii+N1,:)]; % segmento en el dominio temporal
		segF = fft(segT);
        % Multiplicación
        segFO = fft_filtro .* segF;
        % Dominio temporal
        segTO = real(ifft(segFO));
        
        out(ii+1:ii+N1,:) = segTO(N1+1:2*N1,:); % se descartan la primeras N1 muestras debido al aliasing temporal
        pad_block = audio_in(ii+1:ii+N1,:);
        ii = ii+N1;    
    end
end
