clear
clc

%User Defined Properties 
% serialPort = 'COM4';            % define COM port #
plotTitle = 'Sensor';  % plot title
xLabel = 'Tiempo (s)';    % x-axis label
yLabel = 'Ángulos';                % y-axis label
plotGrid = 'on';                % 'off' to turn off grid
min = -180;                     % set y-min
max = 360;                      % set y-max
scrollWidth = 10;               % display period in plot, plot entire data log if <= 0
delay = .0000001;                    % make sure sample faster than resolution

%Define Function Variables
time = 0;
%data = zeros(3,1);
count = 0;
yaw = 0; pitch = 0; % roll = 0;

%Set up Plot
plotGraph = plot(time,yaw,'-r',...
            'LineWidth',2,...
            'MarkerFaceColor','w',...
            'MarkerSize',2);
hold on
plotGraph1 = plot(time,pitch,'-m',...
            'LineWidth',1,...
            'MarkerFaceColor','w',...
            'MarkerSize',2);
% hold on
% plotGraph2 = plot(time,roll,'-b',...
%             'LineWidth',1,...
%             'MarkerFaceColor','w',...
%             'MarkerSize',2);

title(plotTitle,'FontSize',25);
xlabel(xLabel,'FontSize',15);
ylabel(yLabel,'FontSize',15);
axis([0 10 min max]);
grid(plotGrid);
legend('yaw','pitch'); % ,'roll');

%Open Serial COM Port
% s = serial(serialPort,'BaudRate',38400,'DataBits', 8);
% disp('Close Plot to End Session');
% fopen(s);

%Open Bluetooth
% instrhwinfo('Bluetooth')
B = Bluetooth('SENSOR',1);
fopen(B);

%estructuras
azi.value=0; ele.value=0;% incli.value=0;

tic

while ishandle(plotGraph) % && ishandle(plotGraph2) && ishandle(plotGraph1)  %Loop when Plot is Active

% yaw = fscanf(s,'%f'); %Read Data from Serial as Float
      
    count = count + 1;    
    time(count) = toc;    %Extract Elapsed Time in seconds
    
    flushinput(B);
    [~] = strsplit(fscanf(B),',');
    data = strsplit(fscanf(B),',');
    %azi.value = str2double(fscanf(B));
    %pause(0.0001)
    %ele.value = str2double(fscanf(B));
    %pause(0.0001)
    %incli.value = str2double(fscanf(B));
    %pause(0.0001)
    yaw(count) = str2double(data(1)); % azi.value;
    pitch(count) = str2double(data(2)); % ele.value;
    % roll(count) = str2double(data(3)); % incli.value;


    



    %Set Axis according to Scroll Width
    if(scrollWidth > 0)
    set(plotGraph,'XData',time(time > time(count)-scrollWidth),...
        'YData', yaw(1,time > time(count)-scrollWidth));
    set(plotGraph1,'XData',time(time > time(count)-scrollWidth),...
        'YData', pitch(1,time > time(count)-scrollWidth));
%     set(plotGraph2,'XData',time(time > time(count)-scrollWidth),...
%         'YData', roll(1,time > time(count)-scrollWidth));

    axis([time(count)-scrollWidth time(count) min max]);
    else
    set(plotGraph,'XData',time,'YData',yaw(1,:));
    set(plotGraph1,'XData',time,'YData',pitch(1,:));
    % set(plotGraph2,'XData',time,'YData',roll(1,:));

    axis([0 time(count) min max]);
    end

    %Allow MATLAB to Update Plot
    pause(delay);
    
end


%Close Serial COM Port and Delete useless Variables
fclose(B);

clear count dat delay max min plotGraph plotGraph1 plotGraph2 plotGrid...
    plotTitle s scrollWidth serialPort xLabel yLabel;

disp('Session Terminated');

% prompt = 'Export Data? [Y/N]: ';
% str = input(prompt,'s');
% if str == 'Y' || strcmp(str, ' Y') || str == 'y' || strcmp(str, ' y')
%     %export data
%     csvwrite('accelData.txt',data);
%     type accelData.txt;
% else
% end
% 
% clear str prompt;