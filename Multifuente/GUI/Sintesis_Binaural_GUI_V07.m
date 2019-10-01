function varargout = Sintesis_Binaural_GUI_V07(varargin)
% SINTESIS_BINAURAL_GUI_V07 MATLAB code for Sintesis_Binaural_GUI_V07.fig
%      SINTESIS_BINAURAL_GUI_V07, by itself, creates a new SINTESIS_BINAURAL_GUI_V07 or raises the existing
%      singleton*.
%
%      H = SINTESIS_BINAURAL_GUI_V07 returns the handle to a new SINTESIS_BINAURAL_GUI_V07 or the handle to
%      the existing singleton*.
%
%      SINTESIS_BINAURAL_GUI_V07('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SINTESIS_BINAURAL_GUI_V07.M with the given input arguments.
%
%      SINTESIS_BINAURAL_GUI_V07('Property','Value',...) creates a new SINTESIS_BINAURAL_GUI_V07 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Sintesis_Binaural_GUI_V07_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Sintesis_Binaural_GUI_V07_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Sintesis_Binaural_GUI_V07

% Last Modified by GUIDE v2.5 01-Oct-2019 11:34:24

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Sintesis_Binaural_GUI_V07_OpeningFcn, ...
                   'gui_OutputFcn',  @Sintesis_Binaural_GUI_V07_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before Sintesis_Binaural_GUI_V07 is made visible.
function Sintesis_Binaural_GUI_V07_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Sintesis_Binaural_GUI_V07 (see VARARGIN)

% Choose default command line output for Sintesis_Binaural_GUI_V07
handles.output = hObject;

%% INICIALIZACIÓN
%% AGREGAR DIRECTORIOS
addpath ('Bibliotecas/API_MO/API_MO'); % se agrega la carpeta que tiene la funciones del API
addpath (genpath('HRTF'),genpath('Audios'),'Funciones');
handles.directorio_raiz = pwd;
%% BASE DE DATOS FILTROS HRTF
SOFAstart('silent');
handles.sofa.archivo = 'HRIR_L2702_NF150.sofa';
handles.hrtf = SOFAload(handles.sofa.archivo); % Carga los impulsos y metadata de los filtros dentro de una estructura - hrtf b_nh172.sofa - HRIR_L2702_NF100.sofa
handles.hrtf_Fs = handles.hrtf.Data.SamplingRate;
% Indices de las respuestas al impulso
handles.apparentSourceVector = SOFAcalculateAPV(handles.hrtf);
for i=1:length(handles.apparentSourceVector) % si tiene ángulos azimutal negativos los paso a positivos
    if handles.apparentSourceVector(i,1)<0
        handles.apparentSourceVector(i,1) = handles.apparentSourceVector(i,1) + 360;
    end
end
% Respuestas al impulso
handles.hrtf_IR_Left = squeeze(handles.hrtf.Data.IR(:, 1, :)); % se elimina una dimensión
handles.hrtf_IR_Right = squeeze(handles.hrtf.Data.IR(:, 2, :));

set(handles.text_SOFA, 'string', 'HRIR_L2702_NF150.sofa')
set(handles.slider_cf, 'Max', handles.hrtf.API.N)
set(handles.slider_cf, 'SliderStep', [1/(handles.hrtf.API.N-2) , 1/(handles.hrtf.API.N/8-2) ]);

%% CONTROLADOR DE AUDIO
adw = audioDeviceWriter('SampleRate', handles.hrtf_Fs); % se crea el objeto que envía los datos al buffer de salida
release(adw);
adw.Driver = 'DirectSound';
handles.adw = adw;
%% VARIABLES

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Sintesis_Binaural_GUI_V07 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Sintesis_Binaural_GUI_V07_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in boton_cargar_audio.
function boton_cargar_audio_Callback(hObject, eventdata, handles)
% hObject    handle to boton_cargar_audio (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
idx = get(handles.audiopopup, 'Value');
cd 'Audios'
try
    [handles.audio(idx).nombre_audio, directorio] = audioload(handles.hrtf_Fs);

    handles.audio(idx).audioFile = dsp.AudioFileReader(strcat(directorio, handles.audio(idx).nombre_audio)); % archivo de audio
    handles.largo_audio_buffer = length(handles.audio(idx).audioFile());
    % Parametros
    handles.audio(idx).azi.slider = 0;
    handles.audio(idx).ele.slider = 0;
    handles.audio(idx).nivel = 1;
    handles.audio(idx).dB = 0;
    set(handles.static_audio_name, 'String', handles.audio(idx).nombre_audio);
    set(handles.slider_azimuth, 'Value', handles.audio(idx).azi.slider);
    set(handles.edit_azimuth, 'String', num2str(handles.audio(idx).azi.slider));
    set(handles.slider_elevacion, 'Value', handles.audio(idx).ele.slider);
    set(handles.edit_elevacion, 'String', num2str(handles.audio(idx).ele.slider));
    set(handles.sliderNivel, 'Value', handles.audio(idx).dB)
    set(handles.edit_nivel, 'string', num2str(handles.audio(idx).dB))
catch
    try
        handles.audio(idx).nombre_audio = handles.audio(idx).nombre_audio;
    catch
        handles.audio(idx).nombre_audio = ' ';
    end
end
cd (handles.directorio_raiz)

% Update handles structure
guidata(hObject, handles);




% --- Executes on button press in boton_play.
function boton_play_Callback(hObject, eventdata, handles)
% hObject    handle to boton_play (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%% Error check
try
    aud = handles.audio(1).audioFile();
    reset(handles.audio(1).audioFile)
catch
    disp("Primero debe cargar un audio")
    return
end

%% PARAMETROS
source1 = handles.apparentSourceVector(:,1); source2 = handles.apparentSourceVector(:,2);
hrtf_IR_Left = handles.hrtf_IR_Left; hrtf_IR_Right = handles.hrtf_IR_Right;

azi_value = zeros(length(handles.audio),1); 
ele_value = zeros(length(handles.audio),1);
nivel =  zeros(length(handles.audio),1); 
dB =  zeros(length(handles.audio),1); 
%% VECTORES INICIALES
pad_block = zeros(handles.hrtf.API.N,2, length(handles.audio)); % primer bloque de concatenado es cero para OS
posicion_fuente_previa =  zeros(length(handles.audio),1); 
audioIn = zeros(handles.largo_audio_buffer,2,length(handles.audio));
out = zeros(handles.largo_audio_buffer,2,length(handles.audio));
fin = zeros(length(handles.audio), 1);

contador = 0;
detener = 0;
estado_bluetooth = get(handles.static_bluetooth,'String');
final = 0;

% posicion inicial del sensor
if strcmp(estado_bluetooth,'Encendido')
flushinput(handles.sensor);
[~,~] = sensor_read(handles.sensor); % Se ignora la primera lectura
[yaw,pitch] = sensor_read(handles.sensor);
yaw_cero = yaw;
pitch_cero = pitch;
end

% si esta activado grabar
grabar = get(handles.boton_grabar,'Value');

while ((~final) && (detener == 0))  %  ~isDone(handles.audio(1).audioFile) || ~isDone(handles.audio(2).audioFile)
    for i = 1:length(handles.audio)
        [audioIn(:,:,i), fin(i)] = handles.audio(i).audioFile(); % audioIn es el buffer, por default 1024x2 
        if isDone(handles.audio(i).audioFile)
            fin(i)=1;
        end
    end
    if sum(fin)==length(handles.audio)
        final = 1; % avisa que se terminaron todos los audios
    end
    
    %% By pass?
    if get(handles.togglebutton_by_pass,'Value') % Si hay by pass
        for i = 1:length(handles.audio)
            out(:,:,i) = audioIn(:,:,i);
        end
    else % Sin by pass
%     %% SLIDER POSICION FUENTE
%         handles.audio(i).azi.slider = round(get(handles.slider_azimuth,'Value'));
%         handles.audio(i).ele.slider = round(get(handles.slider_elevacion,'Value'));
    %% LECTURA DEL SENSOR
        if strcmp(estado_bluetooth,'Encendido')
            if contador == 3 % cuento 3 loops entre 45-60 ms 
                flushinput(handles.sensor);
            end
            if contador == 3
                [yaw,pitch] = sensor_read(handles.sensor);
                for i = 1:length(handles.audio)
                    [azi_slider(i), ele_slider(i)] = DatosSlider(hObject, i);
                    azi_value(i) = azi_slider(i) - (yaw - yaw_cero); % calcula el delta de la posición
                    ele_value(i) = ele_slider(i) - (pitch - pitch_cero);
                    if azi_value(i) < 0 
                        azi_value(i) = 360 + azi_value(i);
                    end        
                    if azi_value(i) > 359
                       azi_value(i) = azi_value(i) - 360;
                    end
                end
            contador = 0;
            end
        else
            for i = 1:length(handles.audio)     
                [azi_slider(i), ele_slider(i)] = DatosSlider(hObject, i);
                azi_value(i) = azi_slider(i);
                ele_value(i) = ele_slider(i);
            end
        end
    %% FILTRADO CON CONVOLUCIÓN OVERLAP-SAVE    
    for i = 1:length(handles.audio)
        [~,posicion_fuente] = min(pdist2(source1, azi_value(i))+pdist2(source2, ele_value(i))); % busca el valor más proximo de angulo azimut y elevación   
        filtros = [hrtf_IR_Left(posicion_fuente,:)' hrtf_IR_Right(posicion_fuente,:)'];      
        if posicion_fuente_previa(i)~=0 && posicion_fuente ~= posicion_fuente_previa(i) % Con Crossfading
            filtros_previos = [hrtf_IR_Left(posicion_fuente_previa(i),:)' hrtf_IR_Right(posicion_fuente_previa(i),:)']; 
            [out(:,:,i),pad_block(:,:,i)] = partitioned_convolution_OS_CF(audioIn(:,:,i),filtros,pad_block(:,:,i),filtros_previos,get(handles.slider_cf, 'Value'));
        else % Sin crossfading
            [out(:,:,i),pad_block(:,:,i)] = partitioned_convolution_OS_CF(audioIn(:,:,i),filtros,pad_block(:,:,i));
        end
    %% Compensación de Loudness 
        out(:,:,i) = out(:,:,i) * 1.1; % (2.2 para la del MIT) (18 para Ari) (1.1 para HRIR_L2702_NF150) 
    %% ACTUALIZACIÓN VARIABLES
        posicion_fuente_previa(i) = posicion_fuente; % Guardo la posición de la fuente
    end
    contador = contador + 1;
    end
    %% SALIDA CONTROLADOR AUDIO
    for i = 1:length(handles.audio) 
        [nivel(i), dB(i)] = DatosSliderNivel(hObject, i);
        out(:,:,i) = out(:,:,i) * nivel(i); % Nivel de slider para cada pista
    end
    out(:,1) = sum(sum(out(:,1,:),2),3); % Mezcla
    out(:,2) = sum(sum(out(:,2,:),2),3);
    handles.adw([out(:,1),out(:,2)]);
    %% Grabación
    if grabar==1
        handles.afw(out);
    end
    %%ACTUALIZACIÓN VARIABLES
    drawnow % actualizo los callbacks
    detener = get(handles.togglebutton2_detener,'Value'); % boton para detener
end
for i = 1:length(handles.audio)
    reset(handles.audio(i).audioFile);
    handles.audio(i).azi.slider = azi_slider(i);
    handles.audio(i).ele.slider = ele_slider(i);
    handles.audio(i).nivel = nivel(i);
    handles.audio(i).dB = dB(i);
end
if grabar==1
    release(handles.afw);
    movefile(handles.nombre_grabacion,'Grabaciones')
end
 
% Update handles structure
guidata(hObject, handles);



% --- Executes on button press in togglebutton_sensor_bluetooth.
function togglebutton_sensor_bluetooth_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton_sensor_bluetooth (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton_sensor_bluetooth
button_state = get(hObject,'Value');
if button_state == get(hObject,'Max')     % toggle button is pressed
    try
        isobject(handles.sensor);
    catch
        disp('Creando objeto Bluetooth')
        try
            handles.sensor = Bluetooth('SENSOR',1); % CREA EL OBJETO BT
        catch
            disp('Encienda el Bluetooth de la PC')
            return
        end
        disp('Objeto creado')
    end
    try
        fopen(handles.sensor);
    catch
        disp('Encienda el sensor de movimiento y vuelva a intentar')
        return
    end        
    flushinput(handles.sensor);
    fclose(handles.sensor);
    fopen(handles.sensor);
    set(handles.static_bluetooth,'String','Encendido');
    set(hObject,'BackgroundColor','blue')
elseif button_state == get(hObject,'Min')     % toggle button is not pressed
    if strcmp(get(handles.static_bluetooth,'String'),'Encendido')
        fclose(handles.sensor);
        set(handles.static_bluetooth,'String','Apagado');
    end
    set(hObject,'BackgroundColor',[0.94 0.94 0.94])
end
% Update handles structure
guidata(hObject, handles);

% --- Executes on button press in togglebutton2_detener.
function togglebutton2_detener_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton2_detener (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if get(hObject,'Value')==1
    set(hObject,'BackgroundColor','red')
else
    set(hObject,'BackgroundColor',[0.94 0.94 0.94])
end
% Hint: get(hObject,'Value') returns toggle state of togglebutton2_detener


% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
estado_bluetooth = get(handles.static_bluetooth,'String');
% Hint: delete(hObject) closes the figure
 delete(hObject);
% Libero los recursos
cd ('Audios/Temp')
delete *.wav
cd (handles.directorio_raiz)
% if ~isempty(handles.audio.audioFile)
% release(handles.audio.audioFile); 
% end
release(handles.adw);
if strcmp(estado_bluetooth,'Encendido')
fclose(handles.sensor);
end


% --- Executes on button press in togglebutton_by_pass.
function togglebutton_by_pass_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton_by_pass (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if get(hObject,'Value')==1
    set(hObject,'BackgroundColor','green')
else
    set(hObject,'BackgroundColor',[0.94 0.94 0.94])
end
% Hint: get(hObject,'Value') returns toggle state of togglebutton_by_pass


% --- Executes on slider movement.
function slider_azimuth_Callback(hObject, eventdata, handles)
% hObject    handle to slider_azimuth (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
idx = get(handles.audiopopup, 'Value');
handles.audio(idx).azi.slider = round(get(hObject, 'Value'));
set(handles.edit_azimuth,'String',num2str(handles.audio(idx).azi.slider));
% Update handles structure
guidata(hObject, handles);


function [azi, ele] = DatosSlider(hObject, idx)
handles = guidata(hObject);
azi = handles.audio(idx).azi.slider;
ele = handles.audio(idx).ele.slider;

function [nivel, dB] = DatosSliderNivel(hObject, idx)
handles = guidata(hObject);
nivel = handles.audio(idx).nivel;
dB = handles.audio(idx).dB;

% --- Executes during object creation, after setting all properties.
function slider_azimuth_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_azimuth (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider_elevacion_Callback(hObject, eventdata, handles)
% hObject    handle to slider_elevacion (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
idx = get(handles.audiopopup, 'Value');
handles.audio(idx).ele.slider = round(get(hObject, 'Value'));
set(handles.edit_elevacion,'String',num2str(handles.audio(idx).ele.slider));
% Update handles structure
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function slider_elevacion_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_elevacion (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function edit_azimuth_Callback(hObject, eventdata, handles)
% hObject    handle to edit_azimuth (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_azimuth as text
%        str2double(get(hObject,'String')) returns contents of edit_azimuth as a double
idx = get(handles.audiopopup, 'Value');
handles.audio(idx).azi.slider = round(str2double(get(hObject, 'String')));
set(handles.slider_azimuth,'Value',handles.audio(idx).azi.slider);
% Update handles structure
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function edit_azimuth_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_azimuth (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_elevacion_Callback(hObject, eventdata, handles)
% hObject    handle to edit_elevacion (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_elevacion as text
%        str2double(get(hObject,'String')) returns contents of edit_elevacion as a double
idx = get(handles.audiopopup, 'Value');
handles.audio(idx).ele.slider = round(str2double(get(hObject, 'String')));
set(handles.slider_elevacion,'Value',handles.audio(idx).ele.slider);
% Update handles structure
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function edit_elevacion_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_elevacion (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in test_sensor.
function test_sensor_Callback(hObject, eventdata, handles)
% hObject    handle to test_sensor (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
estado_bluetooth = get(handles.static_bluetooth,'String');
if strcmp(estado_bluetooth,'Apagado')
    disp('Primero debe conectar el Sensor Bluetooth')
    return
end
flushinput(handles.sensor);
[~,~] = sensor_read(handles.sensor); % Se ignora la primera lectura
[yaw,pitch] = sensor_read(handles.sensor);
yaw_cero = yaw;
pitch_cero = pitch;

for i=1:20
    flushinput(handles.sensor);
    [yaw,pitch] = sensor_read(handles.sensor);
    azi.value = round(yaw - yaw_cero); % calcula el delta de la posición
    ele.value = round(pitch - pitch_cero);
        if azi.value < 0 
            azi.value = 360 + azi.value;
        end
        if azi.value > 359
            azi.value = azi.value - 360;
        end
    set(handles.text_sensor,'String',['azi : [' num2str(azi.value) '   ]  ele : [' num2str(ele.value) ']']);
    drawnow
    pause(0.1)
end
if strcmp(estado_bluetooth,'Apagado')
fclose(handles.sensor);
end


% --- Executes on button press in boton_grabar.
function boton_grabar_Callback(hObject, eventdata, handles)
% hObject    handle to boton_grabar (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if get(hObject,'Value')==1
    prompt = {'ej. Nombre.wav'};
    dlgtitle = 'Nombre de archivo';
    dims = [1 50];
    definput = {''};
    handles.nombre_grabacion = char(inputdlg(prompt,dlgtitle,dims,definput));
    disp('Espere un momento')
    handles.afw = dsp.AudioFileWriter(handles.nombre_grabacion,'SampleRate',handles.hrtf_Fs); % objeto para grabacion
    disp('listo para grabar')
    set(hObject,'BackgroundColor','red')
else
    set(hObject,'BackgroundColor',[0.94 0.94 0.94])
    release(handles.afw)
end
% Update handles structure
guidata(hObject, handles);
    
% Hint: get(hObject,'Value') returns toggle state of boton_grabar


% --- Executes on selection change in audiopopup.
function audiopopup_Callback(hObject, eventdata, handles)
% hObject    handle to audiopopup (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
idx = get(hObject, 'Value');
try 
    set(handles.static_audio_name, 'String', handles.audio(idx).nombre_audio);
catch
    set(handles.static_audio_name, 'String', ' ');
end
try 
    set(handles.slider_azimuth, 'Value', handles.audio(idx).azi.slider);
    set(handles.edit_azimuth, 'String', num2str(handles.audio(idx).azi.slider));
catch
    set(handles.slider_azimuth, 'Value', 0);
    set(handles.edit_azimuth, 'String', '0');
end
try 
    set(handles.slider_elevacion, 'Value', handles.audio(idx).ele.slider);
    set(handles.edit_elevacion, 'String', num2str(handles.audio(idx).ele.slider));
catch
    set(handles.slider_elevacion, 'Value', 0);
    set(handles.edit_elevacion, 'String', '0');    
end
try 
    set(handles.sliderNivel, 'Value', handles.audio(idx).dB)
    set(handles.edit_nivel, 'string', num2str(handles.audio(idx).dB))
catch
    set(handles.sliderNivel, 'Value', 0);   
    set(handles.edit_nivel, 'string', '0')
end

% Hints: contents = cellstr(get(hObject,'String')) returns audiopopup contents as cell array
%        contents{get(hObject,'Value')} returns selected item from audiopopup
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function audiopopup_CreateFcn(hObject, eventdata, handles)
% hObject    handle to audiopopup (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function sliderNivel_Callback(hObject, eventdata, handles)
% hObject    handle to sliderNivel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
% display(get(hObject, 'Value')) 
idx = get(handles.audiopopup, 'Value');
handles.audio(idx).dB = get(hObject, 'Value');
y = 10.0^((handles.audio(idx).dB)/20);
handles.audio(idx).nivel = y;
set(handles.edit_nivel, 'String', num2str(handles.audio(idx).dB))
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function sliderNivel_CreateFcn(hObject, eventdata, handles)
% hObject    handle to sliderNivel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function edit_nivel_Callback(hObject, eventdata, handles)
% hObject    handle to edit_nivel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_nivel as text
%        str2double(get(hObject,'String')) returns contents of edit_nivel as a double
idx = get(handles.audiopopup, 'Value');
handles.audio(idx).dB = str2double(get(hObject, 'String'));
y = 10.0^((handles.audio(idx).dB)/20);
handles.audio(idx).nivel = y;
set(handles.sliderNivel, 'Value', handles.audio(idx).dB)
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function edit_nivel_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_nivel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in cargar_SOFA.
function cargar_SOFA_Callback(hObject, eventdata, handles)
% hObject    handle to cargar_SOFA (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
try
    archivo = uigetfile('*.sofa');
    handles.sofa.archivo = archivo;
    handles.hrtf = SOFAload(archivo); 
    handles.hrtf_Fs = handles.hrtf.Data.SamplingRate;
    % Indices de las respuestas al impulso
    handles.apparentSourceVector = SOFAcalculateAPV(handles.hrtf);
    for i=1:length(handles.apparentSourceVector) % si tiene ángulos azimutal negativos los paso a positivos
        if handles.apparentSourceVector(i,1)<0
            handles.apparentSourceVector(i,1) = handles.apparentSourceVector(i,1) + 360;
        end
    end
    % Respuestas al impulso
    handles.hrtf_IR_Left = squeeze(handles.hrtf.Data.IR(:, 1, :)); % se elimina una dimensión
    handles.hrtf_IR_Right = squeeze(handles.hrtf.Data.IR(:, 2, :));

    set(handles.text_SOFA, 'string', archivo)
    set(handles.slider_cf, 'Max', handles.hrtf.API.N)
    set(handles.slider_cf, 'SliderStep', [1/(handles.hrtf.API.N-2) , 1/(handles.hrtf.API.N/8-2) ]);
catch
    
end

guidata(hObject, handles);


% --- Executes on slider movement.
function slider_cf_Callback(hObject, eventdata, handles)
% hObject    handle to slider_cf (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
set(handles.edit_cf, 'String', num2str(get(hObject, 'Value')))
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function slider_cf_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_cf (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function edit_cf_Callback(hObject, eventdata, handles)
% hObject    handle to edit_cf (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_cf as text
%        str2double(get(hObject,'String')) returns contents of edit_cf as a double
set(handles.slider_cf, 'Value', round(str2double(get(hObject, 'String'))))
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function edit_cf_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_cf (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
