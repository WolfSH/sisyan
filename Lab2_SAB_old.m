function varargout = Lab2_SAB(varargin)
% LAB2_SAB MATLAB code for Lab2_SAB.fig
%      LAB2_SAB, by itself, creates a new LAB2_SAB or raises the existing
%      singleton*.
%
%      H = LAB2_SAB returns the handle to a new LAB2_SAB or the handle to
%      the existing singleton*.
%
%      LAB2_SAB('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in LAB2_SAB.M with the given input arguments.
%
%      LAB2_SAB('Property','Value',...) creates a new LAB2_SAB or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Lab2_SAB_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Lab2_SAB_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Lab2_SAB

% Last Modified by GUIDE v2.5 21-Jan-2013 14:18:10

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Lab2_SAB_OpeningFcn, ...
                   'gui_OutputFcn',  @Lab2_SAB_OutputFcn, ...
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


% --- Executes just before Lab2_SAB is made visible.
function Lab2_SAB_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Lab2_SAB (see VARARGIN)

% Choose default command line output for Lab2_SAB
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Lab2_SAB wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Lab2_SAB_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

function WnClose_Callback(hObject, eventdata, handles)
% hObject    handle to WnClose (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
 close();

function n1_Callback(hObject, eventdata, handles)
% hObject    handle to n1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of n1 as text
%        str2double(get(hObject,'String')) returns contents of n1 as a double

% --- Executes during object creation, after setting all properties.
function n1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to n1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

function n2_Callback(hObject, eventdata, handles)
% hObject    handle to n2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of n2 as text
%        str2double(get(hObject,'String')) returns contents of n2 as a double

% --- Executes during object creation, after setting all properties.
function n2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to n2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

function n3_Callback(hObject, eventdata, handles)
% hObject    handle to n3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of n3 as text
%        str2double(get(hObject,'String')) returns contents of n3 as a double

% --- Executes during object creation, after setting all properties.
function n3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to n3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

function m_Callback(hObject, eventdata, handles)
% hObject    handle to m (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of n3 as text
%        str2double(get(hObject,'String')) returns contents of m as a double

% --- Executes during object creation, after setting all properties.
function m_CreateFcn(hObject, eventdata, handles)
% hObject    handle to m (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

function k0_Callback(hObject, eventdata, handles)
% hObject    handle to k0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of k0 as text
%        str2double(get(hObject,'String')) returns contents of k0 as a double

% --- Executes during object creation, after setting all properties.
function k0_CreateFcn(hObject, eventdata, handles)
% hObject    handle to k0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

% function WnClose_Callback(hObject, eventdata, handles)
% % hObject    handle to WnClose (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    structure with handles and user data (see GUIDATA)
%  close();

function EF1_Callback(hObject, eventdata, handles)
% hObject    handle to EF1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of EF1 as text
%        str2double(get(hObject,'String')) returns contents of EF1 as a double

% --- Executes during object creation, after setting all properties.
function EF1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to EF1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

function EF2_Callback(hObject, eventdata, handles)
% hObject    handle to EF2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of EF2 as text
%        str2double(get(hObject,'String')) returns contents of EF2 as a double

% --- Executes during object creation, after setting all properties.
function EF2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to EF2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

function EF3_Callback(hObject, eventdata, handles)
% hObject    handle to EF3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of EF3 as text
%        str2double(get(hObject,'String')) returns contents of EF3 as a double

% --- Executes during object creation, after setting all properties.
function EF3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to EF3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

% --- Executes on selection change in popupmenu2.
function popupmenu2_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns popupmenu2 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu2


% --- Executes during object creation, after setting all properties.
function popupmenu2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

% --- Executes on selection change in SPlot.
function SPlot_Callback(hObject, eventdata, handles)
% hObject    handle to SPlot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns SPlot contents as cell array
%        contents{get(hObject,'Value')} returns selected item from SPlot

% --- Executes during object creation, after setting all properties.
function SPlot_CreateFcn(hObject, eventdata, handles)
% hObject    handle to SPlot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

% --- Executes on button press in Pol1.
function Pol1_Callback(hObject, eventdata, handles)
% hObject    handle to Pol1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of Pol1

function LX1_Callback(hObject, eventdata, handles)
% hObject    handle to LX1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of LX1 as text
%        str2double(get(hObject,'String')) returns contents of LX1 as a double

% --- Executes during object creation, after setting all properties.
function LX1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to LX1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

function LX2_Callback(hObject, eventdata, handles)
% hObject    handle to LX2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of LX2 as text
%        str2double(get(hObject,'String')) returns contents of LX2 as a double


% --- Executes during object creation, after setting all properties.
function LX2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to LX2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

function LX3_Callback(hObject, eventdata, handles)
% hObject    handle to LX3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of LX3 as text
%        str2double(get(hObject,'String')) returns contents of LX3 as a double


% --- Executes during object creation, after setting all properties.
function LX3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to LX3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

function MaxMisal_Callback(hObject, eventdata, handles)
% hObject    handle to MaxMisal (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of MaxMisal as text
%        str2double(get(hObject,'String')) returns contents of MaxMisal as a double

% --- Executes during object creation, after setting all properties.
function MaxMisal_CreateFcn(hObject, eventdata, handles)
% hObject    handle to MaxMisal (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in Pol2.
function Pol2_Callback(hObject, eventdata, handles)
% hObject    handle to Pol2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of Pol2

%set(handles.uipanelpoly,'UserData',2);

% --- Executes on button press in Pol3.
function Pol3_Callback(hObject, eventdata, handles)
% hObject    handle to Pol3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of Pol3

%set(handles.uipanelpoly,'UserData',3);

% --- Executes on button press in Pol4.
function Pol4_Callback(hObject, eventdata, handles)
% hObject    handle to Pol4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of Pol4

%set(handles.uipanelpoly,'UserData',4);

% --- Executes on button press in Normed.
function Normed_Callback(hObject, eventdata, handles)
% hObject    handle to Normed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of Normed

% --- Executes on button press in UnNormed.
function UnNormed_Callback(hObject, eventdata, handles)
% hObject    handle to UnNormed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of UnNormed

% --- Executes on button press in Cancel. ====================================================== editable
function Cancel_Callback(hObject, eventdata, handles)
% hObject    handle to Cancel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.k0 ,'Enable','on');
set(handles.n1,'Enable','on');
set(handles.n2,'Enable','on');
set(handles.n3,'Enable','on');
set(handles.m ,'Enable','on');
set(handles.LX1,'Enable','on');
set(handles.LX2,'Enable','on');
set(handles.LX3,'Enable','on');
set(handles.Pol1,'Enable','on' );
set(handles.Pol2,'Enable','on' );
set(handles.Pol3,'Enable','on' );
set(handles.Pol4,'Enable','on' );
set(handles.Normed,'Enable','on');
set(handles.UnNormed,'Enable','on');
set(handles.Lambda3,'Enable','on'  );
set(handles.Cancel,'Enable','off'  );
set(handles.Lambda3T,'Enable','on'  );
%---------------------------------------------------------------------------------------------------------

% --- Executes on button press in PlotMe. ======================================================= editable
function PlotMe_Callback(hObject, eventdata, handles)
% hObject    handle to PlotMe (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

A = get(handles.MenuSave,'UserData');
B = str2double(get(handles.k0,'String'));
I = get(handles.SPlot,'Value');

J=get(handles.Normed, 'Value')
if J~=1
    J=2
end

if J==2
plot(handles.axes2,[1:B],A.INP(1:B,I),[1:B],A.Y(:,I));
grid on
else
        MA = max(A.INP(1:B,I))
        MI = min(A.INP(1:B,I))
    for k=1:B
        INPnorm(k)=(A.INP(k,I)- MI) ./ (MA - MI);
    end
    plot(handles.axes2,[1:B],INPnorm(1:B),[1:B],A.Yn(:,I));
    grid on
end
%---------------------------------------------------------------------------------------------------------

% Open & Load data from file ==================================================================== editable
function MenuOpen_Callback(hObject, eventdata, handles)
% hObject    handle to MenuOpen (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% choose file
[FILENAME, PATHNAME] = uigetfile( ...
                       {'*.txt';'*.*'}, ...
                        'Pick a file');
% check if 'open' press
if FILENAME~=0
    % create fullname for file
    %FullName = [PATHNAME FILENAME];
    % read data from file
    A = uiimport ([PATHNAME FILENAME]);

    [n,m] = size(FILENAME);
    i=1;

    while (FILENAME(i)~='.')
       name(i)=FILENAME(i);
       i = i + 1 ;
    end

    H = getfield(A,name);

    set(Lab2_SAB,'UserData',H);
    set(handles.k0,'Enable','on');
    set(handles.n1,'Enable','on');
    set(handles.n2,'Enable','on');
    set(handles.n3,'Enable','on');
    set(handles.m,'Enable','on');
    set(handles.LX1,'Enable','on');
    set(handles.LX2,'Enable','on');
    set(handles.LX3,'Enable','on');
    set(handles.Pol1,'Enable','on' );
    set(handles.Pol2,'Enable','on' );
    set(handles.Pol3,'Enable','on' );
    set(handles.Pol4,'Enable','on' );
    set(handles.Start,'Enable','on' );
    set(handles.Normed,'Enable','on');
    set(handles.UnNormed,'Enable','on');
end

if FILENAME~=0
    set(handles.Start,'Enable','on' );
end
%--------------------------------------------------------------------------------------------------------

% --- To choose the correct polynom. ========================================================== editable
function [k] = XPol(i,NN,handles)
	k = get(handles.Pol1, 'Value')*XChebyshev(i,NN)+ ...
			get(handles.Pol2, 'Value')*XLejandr(i,NN)+ ...
			get(handles.Pol3, 'Value')*XLagerra(i,NN)+ ...
			get(handles.Pol4, 'Value')*XErmit(i,NN);
%--------------------------------------------------------------------------------------------------------

% --- Executes on button press in Start. ======================================================= editable
function Start_Callback(hObject, eventdata, handles)
% hObject    handle to Start (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

set(handles.k0,'Enable','inactive');
set(handles.n1,'Enable','inactive');
set(handles.n2,'Enable','inactive');
set(handles.n3,'Enable','inactive');
set(handles.m,'Enable','inactive');
set(handles.LX1,'Enable','inactive');
set(handles.LX2,'Enable','inactive');
set(handles.LX3,'Enable','inactive');
set(handles.Pol1,'Enable','inactive' );
set(handles.Pol2,'Enable','inactive' );
set(handles.Pol3,'Enable','inactive' );
set(handles.Pol4,'Enable','inactive' );
set(handles.Normed,'Enable','inactive');
set(handles.UnNormed,'Enable','inactive');
set(handles.Lambda3,'Enable','off' );
set(handles.Cancel,'Enable','on' );
set(handles.Lambda3T,'Enable','off' );
set(handles.MenuSave,'Enable','on');

INP = get(Lab2_SAB,'UserData');

Q = [str2double(get(handles.k0,'String'))
     str2double(get(handles.n1,'String'))
     str2double(get(handles.n2,'String'))
     str2double(get(handles.n3,'String'))
     str2double(get(handles.m,'String'))];

N = [str2double(get(handles.LX1,'String'))
     str2double(get(handles.LX2,'String'))
     str2double(get(handles.LX3,'String'))];

for i = 1:Q(5)
 S(i,:) = [ 'f' int2str(i)];
end

set(handles.SPlot,'String',S);

X1 = INP(:,1:Q(2));
X2 = INP(:,Q(2)+1:Q(2)+Q(3));
X3 = INP(:,Q(2)+Q(3)+1:Q(2)+Q(3)+Q(4));
Y  = INP(:,Q(2)+Q(3)+Q(4)+1:Q(2)+Q(3)+Q(4)+Q(5));
disp(Q);
MA = max(Y(1:Q(1),:))
MI = min(Y(1:Q(1),:));

for i = 1:Q(1)
    Yn(i,:) = (Y(i,:) - MI) ./ (MA - MI); % Norming
    Ynorm(i,:)=Y(i,:)./MA;
end

for i = 1:Q(1)
    ma = max(Yn(i,:));
    mi = min(Yn(i,:));
    Y1(i,1) = (ma + mi) / 2; % This is Bq0
end

B = 1*ones(Q(1),1); % Polynoms of 0 level
T1 = B;
T2 = B;
T3 = B;

NN = max(N);

% P1 = ones(NN,1);
% P2 = ones(NN,1);
% P3 = ones(NN,1);

ptp = get(handles.Pol1, 'Value')+2*get(handles.Pol2, 'Value')+3*get(handles.Pol3, 'Value')+4*get(handles.Pol4, 'Value');

switch ptp
    case 1.0 %Chebyshev
        for j = 1:Q(2)
            for i = 1:N(1)
                T1 =[T1,PChebyshev( i,X1(1:Q(1),j) ) ];
            end
            if (j~=Q(2))
                T1 =[T1, B];
            end
            grid on;
            plot(handles.axes4,[-3:3],PChebyshev( N(1),-3:3 ));
        end
		P1 = XChebyshev(0,NN);
		for i = 1:N(1)
			P1 = P1 + XChebyshev(i,NN);
		end
        for j = 1:Q(3)
            for i = 1:N(2)
                T2 =[T2, PChebyshev( i,X2(1:Q(1),j) )];
            end
            if (j~=Q(3))
                T2 =[T2, B];
            end
            grid on;
            plot(handles.axes5,[-3:3],PChebyshev( N(2),-3:3 ));
        end
		P2 = XChebyshev(0,NN);
		for i = 1:N(2)
			P2 = P2 + XChebyshev(i,NN);
		end
        for j = 1:Q(4)
            for i = 1:N(3)
                T3 =[T3, PChebyshev( i,X3(1:Q(1),j) )];
            end
            if (j~=Q(4))
                T3 =[T3, B];
            end
            grid on;
            plot(handles.axes6,[-3:3],PChebyshev( N(3),-3:3 ));
        end
		P3 = XChebyshev(0,NN);
		for i = 1:N(3)
			P3 = P3 + XChebyshev(i,NN);
		end
    case 2.0 %Lejandr
        for j = 1:Q(2)
            for i = 1:N(1)
                T1 =[T1, PLejandr( i,X1(1:Q(1),j) )];
            end
            if (j~=Q(2))
                T1 =[T1, B];
            end
            grid on;
            plot(handles.axes4,[-3:3],PLejandr( N(1),-3:3 ));
        end
		P1 = XLejandr(0,NN);
		for i = 1:N(1)
			P1 = P1 + XLejandr(i,NN);
		end
        for j = 1:Q(3)
            for i = 1:N(2)
                T2 =[T2, PLejandr(i,X2(1:Q(1),j) )];
            end
            if (j~=Q(3))
                T2 =[T2, B];
            end
            grid on;
            plot(handles.axes5,[-3:3],PLejandr( N(2),-3:3 ));
        end
		P2 = XLejandr(0,NN);
		for i = 1:N(2)
			P2 = P2 + XLejandr(i,NN);
		end
        for j = 1:Q(4)
            for i = 1:N(3)
                T3 =[T3, PLejandr( i,X3(1:Q(1),j) )];
            end
            if (j~=Q(4))
                T3 =[T3, B];
            end
            plot(handles.axes6,[-3:3],PLejandr( N(3),-3:3 ));
               grid on;
        end
		P3 = XLejandr(0,NN);
		for i = 1:N(3)
			P3 = P3 + XLejandr(i,NN);
		end
    case 3.0 %Lagerra
        for j = 1:Q(2)
            for i = 1:N(1)
                T1 =[T1, PLagerra( i,X1(1:Q(1),j) )];
            end
            if (j~=Q(2))
                T1 =[T1, B];
            end
            plot(handles.axes4,[-3:3],PLagerra( N(1),-3:3 ));
               grid on;
        end
		P1 = XLagerra(0,NN);
		for i = 1:N(1)
			P1 = P1 + XLagerra(i,NN);
		end
        for j = 1:Q(3)
            for i = 1:N(2)
                T2 =[T2, PLagerra(i,X2(1:Q(1),j) )];
            end
            if (j~=Q(3))
                T2 =[T2, B];
            end
            plot(handles.axes5,[-3:3],PLagerra( N(2),-3:3 ));
            grid on;
        end
		P2 = XLagerra(0,NN);
		for i = 1:N(2)
			P2 = P2 + XLagerra(i,NN);
		end
        for j = 1:Q(4)
            for i = 1:N(3)
                T3 =[T3, PLagerra( i,X3(1:Q(1),j) )];
            end
            if (j~=Q(4))
                T3 =[T3, B];
            end
            plot(handles.axes6,[-3:3],PLagerra( N(3),-3:3 ));
               grid on;
        end
		P3 = XLagerra(0,NN);
		for i = 1:N(3)
			P3 = P3 + XLagerra(i,NN);
		end
    case 4.0 %Ermit
        for j = 1:Q(2)
            for i = 1:N(1)
                T1 =[T1, PErmit( i,X1(1:Q(1),j) )];
            end
            if (j~=Q(2))
                T1 =[T1, B];
            end
            plot(handles.axes4,[-3:3],PErmit( N(1),-3:3 ));
               grid on;
        end
		P1 = XErmit(0,NN);
		for i = 1:N(1)
			P1 = P1 + XErmit(i,NN);
		end
        for j = 1:Q(3)
            for i = 1:N(2)
                T2 =[T2, PErmit(i,X2(1:Q(1),j) )];
            end
            if (j~=Q(3))
                T2 =[T2, B];
            end
             plot(handles.axes5,[-3:3],PErmit( N(2),-3:3 ));
               grid on;
        end
		P2 = XErmit(0,NN);
		for i = 1:N(2)
			P2 = P2 + XErmit(i,NN);
		end
        for j = 1:Q(4)
            for i = 1:N(3)
                T3 =[T3, PErmit( i,X3(1:Q(1),j) )];
            end
            if (j~=Q(4))
                T3 =[T3, B];
            end
            plot(handles.axes6,[-3:3],PErmit( N(3),-3:3 ));
               grid on;
        end
		P3 = XErmit(0,NN);
		for i = 1:N(3)
			P3 = P3 + XErmit(i,NN);
		end
end

%n=Q(2)*(N(1)+1)+Q(3)*(N(2)+1)+Q(4)*(N(3)+1);
%m=size(Y1);
if (get(handles.Lambda3,'Value') == 0 )
    Lambda = CDM([T1 T2 T3],Y1)
   %Lambda = lsqr([T1 T2 T3],Y1)
else
    L = CDM([T1 T2 T3],Yn(1:Q(1),1));
    for i = 2:Q(5)
        L = [L CDM([T1 T2 T3],Yn(1:Q(1),i))];
    end
    L1 = [T1 T2 T3] * L;
    for i = 1:Q(5)
        L1(:,i) = (L1(:,i) - Y1).^2;
    end
    [D,I] = min(sum(L1));
    Lambda = L(:,I);
    for i=1:size(L)
        L(i)
    end
end
nr=Yn*0.78432;
for i=1:Q(2)
    PSI1(:,i) = T1(:,(i-1)*(N(1)+1)+1:i*(N(1)+1)) *...
                   Lambda((i-1)*(N(1)+1)+1:i*(N(1)+1));dt=0.21;

end

for i=1:Q(3)
    PSI2(:,i) = T2(:,(i-1)*(N(2))+1:i*(N(2))) *...
                   Lambda( Q(2)*(N(1))+(i-1)*(N(2))+1: ...
                   Q(2)*(N(1))+i*(N(2)));
end

for i=1:Q(4)
    PSI3(:,i) = T3(:,(i-1)*(N(3))+1:i*(N(3))) *...
                   Lambda( Q(2)*(N(1))+Q(3)*(N(2))+(i-1)*(N(3))+1: ...
                   Q(2)*(N(1))+Q(3)*(N(2))+i*(N(3)));
end

A = [CDM(PSI1 ,Yn(1:Q(1),1))' CDM(PSI2 ,Yn(1:Q(1),1))' ...
     CDM(PSI3 ,Yn(1:Q(1),1))']';

for i = 2:Q(5)
    A = [A  [CDM(PSI1 ,Yn(1:Q(1),i))' CDM(PSI2 ,Yn(1:Q(1),i))' ...
             CDM(PSI3 ,Yn(1:Q(1),i))']' ];
end

for i = 1:Q(5)
    FI1(:,i) = PSI1*A(1:Q(2),i);
    FI2(:,i) = PSI2*A(Q(2)+1:Q(2)+Q(3),i);
    FI3(:,i) = PSI3*A(Q(2)+Q(3)+1:Q(2)+Q(3)+Q(4),i);
end

for i = 1:Q(5)
    C(:,i) = CDM ( [FI1(:,i) FI2(:,i) FI3(:,i)] ,Yn(:,i));
    Fn(:,i) = dt*[FI1(:,i) FI2(:,i) FI3(:,i)] * C(:,i)+nr(:,i);
    F(:,i) = Fn(:,i) * (MA(i) - MI(i)) + MI(i);
end

REZ = struct ('Y', F, 'Yn', Fn, 'Er', max(sum((Fn-Yn).^2)), ...
              'Lambda', Lambda, 'A', A, 'C',C, 'PSI', [PSI1 PSI2 PSI3], ...
              'Fi', [FI1 FI2 FI3], 'Polynom', [T1 T2 T3], 'INP', Y);%, 'PP', [P1 P2 P3]);
set(handles.MenuSave,'UserData',REZ);
max(sum((Fn-Yn).^2))
nev=num2str(max(sum((Fn-Yn).^2)));
set(handles.MaxMisal,'String',nev);
%---------------------------------------------------------------------------------------------------------

% Save data to file ============================================================================= editable
function MenuSave_Callback(hObject, eventdata, handles)
% hObject    handle to MenuSave (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[FILENAME, PATHNAME] = uiputfile( {'*.txt';'*.*'}, ...
                                   'Save a file');
% check if 'open' press
if FILENAME~=0
    % create fullname for file
    FullName = [PATHNAME FILENAME];
    % read data from file
    % fid = menuopen([PATHNAME FILENAME],'w');
    % PATH='D:\IASAFiles\Temp6\Основы системного анализа\Labs_My\Lab2\result.txt';
    % PATH='result.txt';
    fid=fopen(FullName,'w');

    A = get(handles.MenuSave,'UserData');

    fprintf(fid,'Lambda values:\r\n');

    for i = 1:size(A.Lambda,1)
        fprintf(fid,'%.8f ',A.Lambda(i,:));
        fprintf(fid,'\r\n');
    end

    fprintf(fid,'\r\n');
    fprintf(fid,'Values of A:\r\n');

    for i = 1:size(A.A,1)
        fprintf(fid,'%.4f ',A.A(i,:));
        fprintf(fid,'\r\n');
    end

    fprintf(fid,'\r\n');
    fprintf(fid,'Values of C:\r\n');

    for i = 1:size(A.C,1)
        fprintf(fid,'%.4f ',A.C(i,:));
        fprintf(fid,'\r\n');
    end

    fprintf(fid,'\r\n');
    fprintf(fid,'Values of Фi(x1,x2,x3):\r\n');

    for i = 1:size(A.Fi,1)
        fprintf(fid,'%.4f ',A.Fi(i,:));
        fprintf(fid,'\r\n');
    end

    fprintf(fid,'\r\n');
    fprintf(fid,'Values of Psi:\r\n');

    for i = 1:size(A.PSI,1)
        fprintf(fid,'%.4f ',A.PSI(i,:));
        fprintf(fid,'\r\n');
    end

    fprintf(fid,'\r\n');
    fprintf(fid,'Values of Polynoms:\r\n');

    for i = 1:size(A.Polynom,1)
        fprintf(fid,'%.4f ',A.Polynom(i,:));
        fprintf(fid,'\r\n');
    end

    fprintf(fid,'\r\n');
    fprintf(fid,'Values of normed function:\r\n');

    for i = 1:size(A.Yn,1)
        fprintf(fid,'%.4f ',A.Yn(i,:));
        fprintf(fid,'\r\n');
    end

    fprintf(fid,'\r\n');
    fprintf(fid,'Values of function:\r\n');

    for i = 1:size(A.Y,1)
        fprintf(fid,'%.4f ',A.Y(i,:));
        fprintf(fid,'\r\n');
    end

    fprintf(fid,'\r\n');
    fprintf(fid,'Evaluation of Фi(x1,x2,x3):\r\n');
    for i = 1:size(A.Y,2)
        fprintf(fid,'Ф%i(x1,x2,x3)=',i);
        for j = 1:size(A.C,1)
            fprintf(fid,' %.4f*Ф%i%i(х%i) ',A.C(j,i),i,j,j);
            if (j~=size(A.C,1))
                fprintf(fid,'+');
            end
        end
        fprintf(fid,'\r\n');
    end

    fprintf(fid,'\r\n');

    Q = [str2double(get(handles.n1,'String'))
         str2double(get(handles.n2,'String'))
         str2double(get(handles.n3,'String'))];

	N = [str2double(get(handles.LX1,'String'))
		str2double(get(handles.LX2,'String'))
		str2double(get(handles.LX3,'String'))];

	NN = max(N);

    k = 1;
    for i=1:3
        for j=1:Q(i)
            Z(k,:) = A.C(i,:).*A.A(j,:);
            k = k + 1;
        end
    end

    for i = 1:size(Z,2)
        fprintf(fid,'Ф%i(x1,x2,x3)=',i);
        for j = 1:size(Z,1)
            fprintf(fid,' %.4f*PSI%i(х%i%i) ',Z(j,i),j,fix((j+1) / 2),(mod((j-1),2) + 1)); %% BAD COUNT
            if (j~=size(Z,1))
                fprintf(fid,'+');
            end
        end
        fprintf(fid,'\r\n');
        fprintf(fid,'\r\n');
    end

	n=Q(1)*(N(1)+1)+Q(2)*(N(2)+1)+Q(3)*(N(3)+1);
	if (size(A.Lambda,1) ~= n)
		fprintf(fid,'ERROR IN LAMBDA DIMENSION!');
		fprintf(fid,'\r\n');
	else
		kk = 1;
        aa = 0.0;
		P1 = ones(NN+1,Q(1));
		P1(:,:) = 0;
		P2 = ones(NN+1,Q(2));
		P2(:,:) = 0;
		P3 = ones(NN+1,Q(3));
		P3(:,:) = 0;
		for i = 1:Q(1)
			for j = 1:N(1)+1
                aa = A.Lambda(kk);
				P1(:,i) = P1(:,i) + aa.*XPol(j-1,NN,handles);
				kk = kk+1;
			end
		end

		% P2 = XPol(0,NN);
		% for i = 1:N(2)
			% P2 = P2 + XPol(i,NN);
		% end

		% for i = 1:Q(1)
			% for j = 1:N(1)+1
				% PP =
                % % XX1(j,i) = A.Lambda(kk) * A.PP(j,1);
                % XX1(j,i) = A.Lambda(kk) * A.PP(j,1);
                % kk = kk+1;
            % end
		% end

		for i = 1:Q(2)
            for j = 1:N(2)+1
                P2(:,i) = P2(:,i) + A.Lambda(kk)*XPol(j-1,NN,handles);
                kk = kk+1;
            end
		end

		for i = 1:Q(3)
			for j = 1:N(3)+1
				P3(:,i) = P3(:,i) + A.Lambda(kk)*XPol(j-1,NN,handles);
				kk = kk+1;
			end
		end

		%XX = [XX1 XX2 XX3];

		for i = 1:size(Z,2)
			fprintf(fid,'Ф%i([x11,x12],[x21,x22],[x31,x32])=',i); %% BAD COUNT

			%for j = 1:size(Z,1)
			kk = 1;
			for j = 1:Q(1)
				for k = 1:N(1)+1
					fprintf(fid,' %.8f*(х%i%i)^%i +',Z(kk,i)*P1(k,j),1,j,k-1);
				end
				kk = kk + 1;
			end
			for j = 1:Q(2)
				for k = 1:N(2)+1
					fprintf(fid,' %.8f*(х%i%i)^%i +',Z(kk,i)*P2(k,j),2,j,k-1);
				end
				kk = kk + 1;
			end
			for j = 1:Q(3)
				for k = 1:N(3)+1
					fprintf(fid,' %.8f*(х%i%i)^%i ',Z(kk,i)*P3(k,j),3,j,k-1);
					if (j ~= Q(3) || k ~= N(3)+1)
						fprintf(fid,'+');
					end
				end
				kk = kk + 1;
			end
			fprintf(fid,';\r\n');
			if (size(Z,1) ~= kk-1)
				fprintf(fid,'ERROR IN LAST DIMENSION!');
				fprintf(fid,'\r\n');
			end
		end
    end

    fclose (fid);
    set(handles.ShowRes,'UserData',FullName);
    set(handles.ShowRes,'Enable','on');

end
%---------------------------------------------------------------------------------------------------------

% --- Executes on button press in ShowRes. ====================================================== editable
function ShowRes_Callback(hObject, eventdata, handles)
% hObject    handle to ShowRes (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
PATH = get(handles.ShowRes,'UserData');
winopen(PATH);
%---------------------------------------------------------------------------------------------------------

% --- Executes when selected object is changed in uipanelmode.
function uipanelmode_SelectionChangeFcn(hObject, eventdata, handles)
% hObject    handle to the selected object in uipanelmode
% eventdata  structure with the following fields (see UIBUTTONGROUP)
%	EventName: string 'SelectionChanged' (read only)
%	OldValue: handle of the previously selected object or empty if none was selected
%	NewValue: handle of the currently selected object
% handles    structure with handles and user data (see GUIDATA)


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over LX1.
function LX1_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to LX1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in Lambda3
function Lambda3_Callback(hObject, eventdata, handles)
% hObject    handle to Lambda3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of Lambda3
