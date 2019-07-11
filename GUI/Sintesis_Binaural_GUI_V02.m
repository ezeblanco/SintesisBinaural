function varargout = Sintesis_Binaural_GUI_V02(varargin)
% SINTESIS_BINAURAL_GUI_V02 MATLAB code for Sintesis_Binaural_GUI_V02.fig
%      SINTESIS_BINAURAL_GUI_V02, by itself, creates a new SINTESIS_BINAURAL_GUI_V02 or raises the existing
%      singleton*.
%
%      H = SINTESIS_BINAURAL_GUI_V02 returns the handle to a new SINTESIS_BINAURAL_GUI_V02 or the handle to
%      the existing singleton*.
%
%      SINTESIS_BINAURAL_GUI_V02('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SINTESIS_BINAURAL_GUI_V02.M with the given input arguments.
%
%      SINTESIS_BINAURAL_GUI_V02('Property','Value',...) creates a new SINTESIS_BINAURAL_GUI_V02 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Sintesis_Binaural_GUI_V02_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Sintesis_Binaural_GUI_V02_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Sintesis_Binaural_GUI_V02

% Last Modified by GUIDE v2.5 10-Jul-2019 09:37:42

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Sintesis_Binaural_GUI_V02_OpeningFcn, ...
                   'gui_OutputFcn',  @Sintesis_Binaural_GUI_V02_OutputFcn, ...
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


% --- Executes just before Sintesis_Binaural_GUI_V02 is made visible.
function Sintesis_Binaural_GUI_V02_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Sintesis_Binaural_GUI_V02 (see VARARGIN)

% Choose default command line output for Sintesis_Binaural_GUI_V02
handles.output = hObject;

%% INICIALIZACIÓN
%% AGREGAR DIRECTORIOS
addpath ('Bibliotecas/API_MO/API_MO'); % se agrega la carpeta que tiene la funciones del API
addpath (genpath('HRTF'),genpath('Audios'),'Funciones');
handles.directorio_raiz = pwd;
%% BASE DE DATOS FILTROS HRTF
SOFAstart('silent');
handles.hrtf = SOFAload('HRIR_L2702_NF150.sofa'); % Carga los impulsos y metadata de los filtros dentro de una estructura - hrtf b_nh172.sofa - HRIR_L2702_NF100.sofa
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

%% CONTROLADOR DE AUDIO
adw = audioDeviceWriter('SampleRate', handles.hrtf_Fs); % se crea el objeto que envía los datos al buffer de salida
release(adw);
adw.Driver = 'DirectSound';
handles.adw = adw;
%% VARIABLES
handles.audioFile = [];

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Sintesis_Binaural_GUI_V02 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Sintesis_Binaural_GUI_V02_OutputFcn(hObject, eventdata, handles) 
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
cd 'Audios'
[nombre_audio,directorio] = uigetfile('*.*','Elija el archivo de audio');
% cd (directorio)
[audio,Fs] = audioread(strcat(directorio,nombre_audio));
formato = strsplit(nombre_audio,'.');
formato = strlength(formato(length(formato)))+1;

if Fs ~= handles.hrtf_Fs % si la Fs del audio no coincide con los filtros
    [P,Q] = rat(handles.hrtf_Fs/Fs);
    audio = resample(audio * 0.9,P,Q); % 0.9 constante para que no clipee cuando convierta
    if size(audio,2)==1 % si es mono
    audio = [audio audio]; % hago dos canales
    end
    nombre_audio = [nombre_audio(1,1:length(nombre_audio)-formato) '_' num2str(handles.hrtf_Fs) '_2.wav'];
    audiowrite(nombre_audio,audio,handles.hrtf_Fs);
    
elseif size(audio,2)==1 % si es mono
    audio = [audio audio]; % hago dos canales
    nombre_audio = [nombre_audio(1,1:length(nombre_audio)-formato) '_2.wav']; % num2str(handles.hrtf_Fs)
    audiowrite(nombre_audio,audio,handles.hrtf_Fs);
end

handles.audioFile = dsp.AudioFileReader(strcat(directorio,nombre_audio)); % archivo de audio
handles.largo_audio_buffer = length(handles.audioFile());

cd (handles.directorio_raiz)
set(handles.static_audio_name, 'String', nombre_audio);

% Update handles structure
guidata(hObject, handles);



% --- Executes on button press in boton_play.
function boton_play_Callback(hObject, eventdata, handles)
% hObject    handle to boton_play (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%% PARAMETROS
azi.value = 0; % se eligen los angulos azimutal y elevación iniciales del controlador
ele.value = 0;
%% VECTORES INICIALES
pad_block = zeros(handles.hrtf.API.N,2); % primer bloque de concatenado es cero para OS
posicion_fuente_previa = 0;
contador = 0;
detener = 0;
estado_bluetooth = get(handles.static_bluetooth,'String');

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

while ~isDone(handles.audioFile) && (detener == 0)
    audioIn = handles.audioFile(); % audioIn es el buffer, por default 1024x2 
    %% By pass?
    if get(handles.togglebutton_by_pass,'Value') % Si hay by pass
        out = audioIn;
    else % Sin by pass
    %% SLIDER POSICION FUENTE
        azi.slider = round(get(handles.slider_azimuth,'Value'));
        ele.slider = round(get(handles.slider_elevacion,'Value'));
    %% LECTURA DEL SENSOR
        if strcmp(estado_bluetooth,'Encendido')
            if contador == 3 % cuento 3 loops entre 45-60 ms 
                flushinput(handles.sensor);
            end
            if contador == 3
            [yaw,pitch] = sensor_read(handles.sensor);
            azi.value = round(azi.slider - (yaw - yaw_cero)); % calcula el delta de la posición
            ele.value = round(ele.slider - (pitch - pitch_cero));
                if azi.value < 0 
                    azi.value = 360 + azi.value;
                end        
                if azi.value > 359
                   azi.value = azi.value - 360;
                end
            contador = 0;
            end
        else
            azi.value = round(azi.slider);
            ele.value = round(ele.slider);
        end
    %% FILTRADO CON CONVOLUCIÓN OVERLAP-SAVE
        [~,posicion_fuente] = min(pdist2(handles.apparentSourceVector(:,1), azi.value)+pdist2(handles.apparentSourceVector(:,2), ele.value)); % busca el valor más proximo de angulo azimut y elevación   
        filtros = [handles.hrtf_IR_Left(posicion_fuente,:)' handles.hrtf_IR_Right(posicion_fuente,:)'];      
        if posicion_fuente_previa~=0 && posicion_fuente ~= posicion_fuente_previa % Con Crossfading
            filtros_previos = [handles.hrtf_IR_Left(posicion_fuente_previa,:)' handles.hrtf_IR_Right(posicion_fuente_previa,:)']; 
            [out,pad_block] = partitioned_convolution_OS_CF(audioIn,filtros,pad_block,filtros_previos);
        else % Sin crossfading
            [out,pad_block] = partitioned_convolution_OS_CF(audioIn,filtros,pad_block);
        end
    %% Compensación de Loudness 
    out = out * 1.1; % (2.2 para la del MIT) (18 para Ari) (1.1 para HRIR_L2702_NF150) 

    %% ACTUALIZACIÓN VARIABLES
    posicion_fuente_previa = posicion_fuente; % Guardo la posición de la fuente
    contador = contador + 1;
    end
    %% SALIDA CONTROLADOR AUDIO
    handles.adw([out(:,1),out(:,2)]);
    %% Grabación
    if grabar==1
        handles.afw(out);
    end
    %%ACTUALIZACIÓN VARIABLES
    drawnow % actualizo los callbacks
    detener = get(handles.togglebutton2_detener,'Value'); % boton para detener
end
reset(handles.audioFile);
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
        handles.sensor = Bluetooth('SENSOR',1); % CREA EL OBJETO BT
        disp('Objeto creado')
    end
    
    fopen(handles.sensor);
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
if ~isempty(handles.audioFile)
release(handles.audioFile); 
end
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
set(handles.edit_azimuth,'String',num2str(round(get(hObject,'Value'))));

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
set(handles.edit_elevacion,'String',num2str(round(get(hObject,'Value'))));

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
set(handles.slider_azimuth,'Value',str2double(get(hObject,'String')));

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
set(handles.slider_elevacion,'Value',str2double(get(hObject,'String')));

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
fopen(handles.sensor);
flushinput(handles.sensor);
fclose(handles.sensor);
fopen(handles.sensor);
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
