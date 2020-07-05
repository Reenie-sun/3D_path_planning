function varargout = UI_pathplanning(varargin)
% UI_PATHPLANNING MATLAB code for UI_pathplanning.fig
%      UI_PATHPLANNING, by itself, creates a new UI_PATHPLANNING or raises the existing
%      singleton*.
%
%      H = UI_PATHPLANNING returns the handle to a new UI_PATHPLANNING or the handle to
%      the existing singleton*.
%
%      UI_PATHPLANNING('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in UI_PATHPLANNING.M with the given input arguments.
%
%      UI_PATHPLANNING('Property','Value',...) creates a new UI_PATHPLANNING or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before UI_pathplanning_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to UI_pathplanning_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help UI_pathplanning

% Last Modified by GUIDE v2.5 21-Nov-2017 11:32:47

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @UI_pathplanning_OpeningFcn, ...
                   'gui_OutputFcn',  @UI_pathplanning_OutputFcn, ...
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


% --- Executes just before UI_pathplanning is made visible.
function UI_pathplanning_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to UI_pathplanning (see VARARGIN)

% Choose default command line output for UI_pathplanning
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes UI_pathplanning wait for user response (see UIRESUME)
% uiwait(handles.figure1);

global x_2d_s;
global y_2d_s;
global z_2d_s;

global vx_new;
global vy_new;
global xn_new;
global yn_new;

global r_sq;

global xc;
global yc;

global N_s;

global buttonCount;

global xy_opt_path;
global temp_xy_opt_path;

global xc_total;
global yc_total;
global mouseClickCount;

global idx;

global vx_old;
global vy_old;

global c_r;

global vx1;
global vy1;
global vx2;
global vy2;

global samsiteX;
global samsiteY;
global samsiteR;

global d_time;
global d_velo;
global path_length;

global drone_count;
global part_length_all;
global ttemp_xy_path_all;

ttemp_xy_path_all = [];

part_length_all = [];

path_length = [];

d_time = 0;
d_velo = 0;

vx1 = [];
vy1 = [];
vx2 = [];
vy2 = [];

% vx_new = [];
% vy_new = [];
% xn_new = [];
% yn_new = [];

r_sq = [];

xc = [];
yc = [];
xc_total = xc;
yc_total = yc;

idx = [];

vx_old = [];
vy_old = [];

samsiteX = 0;
samsiteY = 0;
samsiteR = 0;

mouseClickCount = 0;
drone_count = 0;

set(handles.edit_drone_time, 'String', '2');
set(handles.edit_drone_velocity, 'String', '75');
set(handles.editHigh, 'String', '700');
set(handles.editSamsiteR, 'String', '5');

% 여기서 부터 추가 %%%%%%%%%%%%%%%%%%%%    1   %%%%%%%%%%%%%%%%
buttonCount = 0;

[Z, ref] = dted('n37.dt0');

N_s = 60;
x_s = linspace(0, N_s,N_s);
y_s = linspace(0, N_s,N_s);

x_2d_s = repmat(x_s,N_s,1);
y_2d_s = repmat(y_s',1,N_s);

x_1d_s = zeros(1,N_s*N_s);
y_1d_s = zeros(1,N_s*N_s);
z_1d_s = zeros(1,N_s*N_s);
z_2d_s = zeros(N_s,N_s);

cnt = 1;
for i = 1: N_s %1자로 만드는부분
    for j = 1: N_s
        x_1d_s(cnt) = x_2d_s(i,j);
        y_1d_s(cnt) = y_2d_s(i,j);
        z_1d_s(cnt) = Z(i,j);
        z_2d_s(i,j) = Z(i,j);
        cnt = cnt+1;
    end
end

%1단계 등고선지도 획득
%figure(1);
set(gcf,'numbertitle','off','name', '등고선지도');
contour(x_2d_s,y_2d_s,z_2d_s,'ShowText','on');
xlabel('km');
ylabel('km');

% 여기 까지 추가 %%%%%%%%%%%%%%%%%%%  1   %%%%%%%%%%%%%%%%%%%%%%%


% --- Outputs from this function are returned to the command line.
function varargout = UI_pathplanning_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;





function editSamsiteX_Callback(hObject, eventdata, handles)
% hObject    handle to editSamsiteX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editSamsiteX as text
%        str2double(get(hObject,'String')) returns contents of editSamsiteX as a double


% --- Executes during object creation, after setting all properties.
function editSamsiteX_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editSamsiteX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editSamsiteY_Callback(hObject, eventdata, handles)
% hObject    handle to editSamsiteY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editSamsiteY as text
%        str2double(get(hObject,'String')) returns contents of editSamsiteY as a double


% --- Executes during object creation, after setting all properties.
function editSamsiteY_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editSamsiteY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbuttonSamsite.
function pushbuttonSamsite_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonSamsite (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global vx_new;
global vy_new;
global xn_new;
global yn_new;

global x_2d_s;
global y_2d_s;
global z_2d_s;

global N_s;

global r_sq;

global vx1;
global vy1;
global vx2;
global vy2;

global buttonCount;

global xc_total;
global yc_total;
global mouseClickCount;

global idx;

global c_r;

global vx_old;
global vy_old;

global samsiteX;
global samsiteY;
global samsiteR;


buttonCount = 1;

% figure 지우기 샘사이트 지우기
% cla;

% samsiteX = str2double(get(handles.editSamsiteX, 'String'));
% samsiteY = str2double(get(handles.editSamsiteY, 'String'));
samsiteR = str2double(get(handles.editSamsiteR, 'String'));


% 샘사이트 마우스 입력
    
% 마우스로 좌표 넣는 부분
% 왼쪽 마우스 : 1반환, 오른쪽 마우스 : 3 반환
% samsiteX = [];
% samsiteY = [];

% 마우스로 지점 클릭 -> 우클릭시 좌표 입력 종료
button = 1;



% figure(1);
set(gcf,'numbertitle','off','name', '샘사이트반영');

th = 0:0.01:2*pi; %원을 그리기위한 각도 배열


% vx1 = vx_new(1,:); 
% vy1 = vy_new(1,:);

contour(x_2d_s,y_2d_s,z_2d_s,'ShowText','on');
xlabel('km');
ylabel('km');
hold on;




while 1

    [xMouse, yMouse, button] = ginput(1);

    if button == 3
        break;

    else
        
        mouseClickCount = mouseClickCount + 1;

%         samsiteX(mouseClickCount) = xMouse;
%         samsiteY(mouseClickCount) = yMouse;
        
        samsiteX(mouseClickCount) = xMouse;
        samsiteY(mouseClickCount) = yMouse;
        
        
        c_cx = samsiteX(mouseClickCount); c_cy = samsiteY(mouseClickCount); c_r = samsiteR;
%         c_cx = samsiteX; c_cy = samsiteY; c_r = samsiteR;
        
        xc = c_r*cos(th)+c_cx; 
        yc = c_r*sin(th)+c_cy;
        
        h = fill(xc,yc,'red');
        set(h,'facealpha',.5);
        
        vx1 = vx_new(1,:); 
        vy1 = vy_new(1,:);
        
        r_sq = (vx1-c_cx).^2+(vy1-c_cy).^2;
        idx1 = find((r_sq < c_r^2));

        vx2 = vx_new(2,:);
        vy2 = vy_new(2,:);

        r_sq = (vx2-c_cx).^2+(vy2-c_cy).^2;
        idx2 = find((r_sq < c_r^2));

        idx = union(idx1,idx2);


        vx_new(:,idx) = [];
        vy_new(:,idx) = [];
        
        xc_total{mouseClickCount} = xc;
        yc_total{mouseClickCount} = yc;
        
%         disp(samsiteX);

    end

end

vx_old = vx_new;
vy_old = vy_new;

% figure 내용 삭제
cla;

set(gcf,'numbertitle','off','name', '샘사이트반영');

contour(x_2d_s,y_2d_s,z_2d_s,'ShowText','on');
xlabel('km');
ylabel('km');
hold on;


for i = 1 : mouseClickCount
   
    c_cx = samsiteX(i); c_cy = samsiteY(i); c_r = samsiteR;
%         c_cx = samsiteX; c_cy = samsiteY; c_r = samsiteR;
        
    xc = c_r*cos(th)+c_cx; 
    yc = c_r*sin(th)+c_cy;

    h = fill(xc,yc,'red');
    set(h,'facealpha',.5);
    hold on;
    
end


% contour(x_2d_s,y_2d_s,z_2d_s,'ShowText','on');
% xlabel('km');
% ylabel('km');
% hold on;
plot(vx_new,vy_new,'b.-');
plot(xn_new,yn_new,'r.');
% h = fill(xc,yc,'red');
% set(h,'facealpha',.5)
axis([0 N_s 0 N_s]);


% --- Executes on button press in pushbuttonNext.
function pushbuttonNext_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonNext (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global x_2d_s;
global y_2d_s;
global z_2d_s;

global vx_new;
global vy_new;
global xn_new;
global yn_new;

global xpoint;
global ypoint;

global r_sq;

global xc;
global yc;

global N_s;

global buttonCount;

global xy_opt_path;

global pointCount;  % 마우스로 좌표 찍는 것 개수 세는 변수
global temp_xy_opt_path;
global rootCount;

global xc_total;
global yc_total;
global mouseClickCount;

global idx;

global vx_old;
global vy_old;

global c_r;

global vx1;
global vy1;
global vx2;
global vy2;

global samsiteX;
global samsiteY;
global samsiteR;

global path_length_all;
global path_length;

global d_time;
global d_velo;
global drone_s;

global xy_opt_path_plus;

global drone_count;
global sol_xy_op_path;

global ttemp_xy_path_all;

global order;

global xy_count;
global xct;

global part_length_all;

% drone_count = 0;

buttonCount = buttonCount + 1;
   
if(buttonCount == 1)
    %3단계 운항가능 경로 표기

    %figure(1);
    set(gcf,'numbertitle','off','name', '경로표기');
    contour(x_2d_s,y_2d_s,z_2d_s,'ShowText','on');
    xlabel('km');
    ylabel('km');
    hold on;
    
%     vx_new(:,idx) = [];
%     vy_new(:,idx) = [];
    
    %%%%%%%%%%%% 새로 추가
    if(isempty(vx_old) ~= 0)
    
        for i = 1 : length(vx_old)

            vx_old = [];
            vy_old = [];

        end

    end
    
    if(mouseClickCount ~= 0)
    
        th = 0:0.01:2*pi; %원을 그리기위한 각도 배열
    
        for i = 1 : mouseClickCount

            c_cx = samsiteX(i); c_cy = samsiteY(i); c_r = samsiteR;

            xc = c_r*cos(th)+c_cx; 
            yc = c_r*sin(th)+c_cy;

            h = fill(xc,yc,'red');
            set(h,'facealpha',.5);

            vx1 = vx_new(1,:); 
            vy1 = vy_new(1,:);

            r_sq = (vx1-c_cx).^2+(vy1-c_cy).^2;
            idx1 = find((r_sq < c_r^2));

            vx2 = vx_new(2,:);
            vy2 = vy_new(2,:);

            r_sq = (vx2-c_cx).^2+(vy2-c_cy).^2;
            idx2 = find((r_sq < c_r^2));

            idx = union(idx1,idx2);


            vx_new(:,idx) = [];
            vy_new(:,idx) = [];


        end

    end
    
    %%%%%%%%%%%%%%%
    
    if(~isempty(vx_old) && ~isempty(vy_old))
       
%         vx_new = vx_old;
%         vy_new = vy_old;
        
        c_r = 1;
        
        for i = 1 : length(xn_new)
            % 1열의 데이터 획득
            vx1 = vx_new(1,:); 
            vy1 = vy_new(1,:);

            % 2열의 데이터 획득
            vx2 = vx_new(2,:);
            vy2 = vy_new(2,:);

            r_sq = (vx1-xn_new(i)).^2+(vy1-yn_new(i)).^2;
            idx1 = find((r_sq < c_r^2));

            r_sq = (vx2-xn_new(i)).^2+(vy2-yn_new(i)).^2;
            idx2 = find((r_sq < c_r^2));

            idx = union(idx1,idx2);


            vx_new(:,idx) = [];
            vy_new(:,idx) = [];

        end
        
    end
    
    plot(vx_new,vy_new,'b.-');
    plot(xn_new,yn_new,'r.');
    axis([0 N_s 0 N_s]);
    
elseif(buttonCount == 2)
    %5단계 최종경로
    
    % 마우스로 좌표 넣는 부분
    % 왼쪽 마우스 : 1반환, 오른쪽 마우스 : 3 반환
    xpoint = [];
    ypoint = [];

    % 마우스로 지점 클릭 -> 우클릭시 좌표 입력 종료
    button = 1;

    posCount = 0;
    pointCount = 0;

    while 1

        [xMouse, yMouse, button] = ginput(1);

        posCount = posCount + 1;

        if button == 3
            break

        else
            plot(xMouse, yMouse, 'rx', 'MarkerSize', 15);

            if pointCount == 0
                c = '시작 지점';
                text(xMouse + 1, yMouse, c, 'FontSize', 15, 'Color', 'red', 'FontWeight', 'bold');
            else
                c = strcat(int2str(posCount-1), '번 지점');
                text(xMouse + 1, yMouse, c, 'FontSize', 15, 'Color', 'red', 'FontWeight', 'bold');
            end

            pointCount = pointCount + 1;

            xpoint(pointCount) = xMouse;
            ypoint(pointCount) = yMouse;

        end

    end

    xpoint = xpoint';
    ypoint = ypoint';
    
elseif(buttonCount == 3)
%     order = tsp(xpoint,ypoint);

    if(length(xpoint) > 3)
        order = tsp(xpoint,ypoint);
        
    else
        order = [1 2];
        
    end

    %figure(1);
    set(gcf,'numbertitle','off','name', '최종경로');
    hold on;

    lgd = legend('출발점','도착점');
    lgd.AutoUpdate = 'off';
    contour(x_2d_s,y_2d_s,z_2d_s,'ShowText','on');
    xlabel('km');
    ylabel('km');

    plot(vx_new,vy_new,'b.-');
    plot(xn_new,yn_new,'r.');
    h = fill(xc,yc,'red');
    set(h,'facealpha',.5)
    axis([0 N_s 0 N_s]);
%     order;

    assignin('base', 'order', order);
    
    path_length_all = 0;
%     part_length_all = [];
    
    for mainloof = 1 : length(order)-1
        
        xy_start = [xpoint(order(mainloof)) ypoint(order(mainloof))];
        xy_dest = [xpoint(order(mainloof+1)) ypoint(order(mainloof+1))];
        
        vx_all = vx_new(:);
        vy_all = vy_new(:);

        dr = kron(ones(length(vx_all),1),xy_start)-[vx_all vy_all];
        [min_val,min_id] = min(sum(dr.^2,2));

        vx_new = [vx_new [xy_start(1); vx_all(min_id)]];
        vy_new = [vy_new [xy_start(2); vy_all(min_id)]];

        dr = kron(ones(length(vx_all),1),xy_dest)-[vx_all vy_all];
        [min_val,min_id] = min(sum(dr.^2,2));

        vx_new = [vx_new [xy_dest(1); vx_all(min_id)]];
        vy_new = [vy_new [xy_dest(2); vy_all(min_id)]];

        xy_all = unique([vx_new(:) vy_new(:)],'rows');
        dv = [vx_new(1,:); vy_new(1,:)] - [vx_new(2,:); vy_new(2,:)];
        edge_dist = sqrt(sum(dv.^2));

        G = sparse(size(xy_all,1),size(xy_all,1));

        for kdx = 1:length(edge_dist)
            xy_s = [vx_new(1,kdx) vy_new(1,kdx)];
            idx = find(sum((xy_all-kron(ones(size(xy_all,1),1),xy_s)).^2,2)==0);
            xy_d = [vx_new(2,kdx) vy_new(2,kdx)];
            jdx = find(sum((xy_all-kron(ones(size(xy_all,1),1),xy_d)).^2,2)==0);
            G(idx,jdx) = edge_dist(kdx);
            G(jdx,idx) = edge_dist(kdx);
        end

        st_idx = find(sum((xy_all-kron(ones(size(xy_all,1),1),xy_start)).^2,2)==0);
        dest_idx = find(sum((xy_all-kron(ones(size(xy_all,1),1),xy_dest)).^2,2)==0);

        [dist,paths,pred] = graphshortestpath(G,st_idx,dest_idx);
        xy_opt_path = xy_all(paths,:);
        % for i = 1 : length(xy_opt_path)
        %    
        % end
        %1열의 데이터 획득
        d_r = 1.4;
        d_r2 = 0.9;
        xd = xy_opt_path(:,1); 
        yd = xy_opt_path(:,2);
        trues = zeros(1,length(xy_opt_path));
        for i = 1 : length(xy_opt_path)
            r_sq = (xd(i)-xn_new).^2+(yd(i)-yn_new).^2;
            idx1 = find((r_sq < d_r^2));
            r_sq = (xd(i)-xc).^2+(yd(i)-yc).^2;
            idx2 = find((r_sq < d_r2^2));

            if (isempty(idx1)==1) && (isempty(idx2)==1)
                trues(i) = 1;
            end
        end

        plot(xy_start(1),xy_start(2),'S',xy_dest(1),xy_dest(2),'D','MarkerSize',10, 'MarkerFaceColor','red');

        %c = '시작 지점';
        %text(xpoint(1) - 5, ypoint(1) - 3, c, 'FontSize', 13, 'FontWeight', 'bold');

        plot(xy_opt_path(:,1),xy_opt_path(:,2), '-k', 'LineWidth',3);
        
        %%%%%%%%%%%%%%
        
%         temp_xy_opt_path : cell data     
        
        
        % 직선 경로 데이터
        ttemp_xy_path_all{mainloof} = xy_opt_path;
        
        rootCount = 0;
        
        
        
        % 전체 경로 길이 구간별 계산
        path1 = 0;
                
        for i = 1 : length(xy_opt_path)-1
        
            path_part = sqrt((xy_opt_path(i,1) - xy_opt_path(i+1,1))^2 + (xy_opt_path(i,2) - xy_opt_path(i+1,2))^2);
            path1 = path_part + path1;
            part_length(i) = path_part;
                        
        end
        
        path_length(mainloof) = path1;
        part_length_all{mainloof} = part_length;
        
%         path_length_all = path_length(mainloof) + path_length_all;
        path_length_all = path1 + path_length_all;
        
    end
    
    set(handles.edit_drone_all_way, 'String', sprintf('%s [km]', num2str(path_length_all)));
    drone_time = path_length_all / d_velo;
    set(handles.edit_drone_time, 'String', sprintf('%s [h]', num2str(drone_time)));
    
    assignin('base', 'path_length', path_length);
    assignin('base', 'path_length_all', path_length_all);
    disp('전체 직선 경로 길이');
    path_length_all
        
    assignin('base', 'xpoint', xpoint);
    assignin('base', 'ypoint', ypoint);
    assignin('base', 'ttemp_xy_path_all', ttemp_xy_path_all); 
                
elseif(buttonCount == 4)
    
        
%%%%%%%%%%%%% 경로 곡선으로 변경

% 전체 곡선 경로 길이
curve_path_all = 0;

bank_v_all = [];

    for i = 1 : length(ttemp_xy_path_all)
        
        p = 2;
        
        temp_xy_opt_path2 = ttemp_xy_path_all{i};
        
        s_length = part_length_all{i};
        
        % 곡선 경로 짧은 길이
        curve_part_path = 0;
        % 곡선 경로 order 별 길이
        curve_part_length = s_length(1) + s_length(2)/2  + s_length(end-1)/2 + s_length(end);
        
        bank_v_part = [];
        
        x1 = (temp_xy_opt_path2(2,1)+temp_xy_opt_path2(3,1))/2;y1 = (temp_xy_opt_path2(2,2)+temp_xy_opt_path2(3,2))/2;
        plot([temp_xy_opt_path2(1,1) temp_xy_opt_path2(2,1) x1], [temp_xy_opt_path2(1,2) temp_xy_opt_path2(2,2) y1],'red','LineWidth',3);
        
        x1 = (temp_xy_opt_path2(end-2,1)+temp_xy_opt_path2(end-1,1))/2;y1 = (temp_xy_opt_path2(end-2,2)+temp_xy_opt_path2(end-1,2))/2;
        plot([ x1 temp_xy_opt_path2(end-1,1) temp_xy_opt_path2(end,1)], [ y1 temp_xy_opt_path2(end-1,2) temp_xy_opt_path2(end,2)],'red','LineWidth',3);

        while p < length(temp_xy_opt_path2)-2           
            if length(temp_xy_opt_path2) > 1
                if p == length(temp_xy_opt_path2)-1
                    x1 = (temp_xy_opt_path2(p,1)+temp_xy_opt_path2(p+1,1))/2;y1 = (temp_xy_opt_path2(p,2)+temp_xy_opt_path2(p+1,2))/2;   
                    x2 = temp_xy_opt_path2(p+1,1); y2 = temp_xy_opt_path2(p+1,2) ; 
                    x3 = (temp_xy_opt_path2(end,1)+temp_xy_opt_path2(p+1,1))/2; y3  = (temp_xy_opt_path2(end,2)+temp_xy_opt_path2(p+1,2))/2;

                else
                    x1 = (temp_xy_opt_path2(p,1)+temp_xy_opt_path2(p+1,1))/2;y1 = (temp_xy_opt_path2(p,2)+temp_xy_opt_path2(p+1,2))/2;   
                    x2 = temp_xy_opt_path2(p+1,1); y2 = temp_xy_opt_path2(p+1,2) ; 
                    x3 = (temp_xy_opt_path2(p+2,1)+temp_xy_opt_path2(p+1,1))/2; y3  = (temp_xy_opt_path2(p+2,2)+temp_xy_opt_path2(p+1,2))/2; 
                    
                end
                
                % [km/h] 뱅크각
                bank_v = sqrt( (sqrt((x1 - x2)^2 + (y1 - y2)^2)) * 68578 ) * 1.6;
                bank_v_part(end+1) = bank_v;
                
                a = [x1, y1, 1;    x2, y2, 1;    x3, y3, 1];
                d = [x1^2+y1^2, y1, 1;    x2^2+y2^2, y2, 1;    x3^2+y3^2, y3, 1];
                
                e = [x1^2+y1^2, x1, 1;    x2^2+y2^2, x2, 1;    x3^2+y3^2, x3, 1];
                f = [x1^2+y1^2, x1, y1;    x2^2+y2^2, x2, y2;    x3^2+y3^2, x3, y3];

                centerx = -(det(-d)/(2 * det(a)));
                centery = -(det(e)/(2 * det(a)));
                radius = sqrt( ((det(-d)^2 + det(e)^2) / (4*det(a)^2)) - (det(-f)/det(a)));

                seta1 = acos((x1-centerx)/radius)*180/pi;
                seta2 = acos((x2-centerx)/radius)*180/pi;
                seta3 = acos((x3-centerx)/radius)*180/pi;

                if centery > y1
                    seta1 = 360 - seta1;
                end

                if centery > y2
                    seta2 = 360 - seta2;
                end

                if centery > y3
                    seta3 = 360 - seta3;
                end

                thtemp = [];
                
                if( ((x1 == x2)&&(x2==x3)) || ((y1 == y2)&&(y2==y3)) )
                    plot([x1 x2 x3], [y1 y2 y3], 'red','LineWidth',3);
                    
                    curve_part_path = sqrt((x1 - x2)^2 + (y1 - y2)^2) + sqrt((x3 - x2)^2 + (y3 - y2)^2);
                
                else
                    resulty = (((y3-y1)/(x3-x1)) * (x2-x1)) + y1;

                    if x1 > x3 %시작점이 오쪽에 있을때
                        if y1 > y3 %시작점이 오위 y값이 크면 왼쪽에있는거
                            if resulty > y2 %오른쪽에 있네
                                if seta1 < 180
                                    tempangle1 = 0 : 0.01 : seta1/180*pi;
                                    tempangle2 = seta3/180*pi : 0.01 : 2*pi;
                                    thtemp = [tempangle2,tempangle1];
                                    
                                    curve_th = abs(2*pi-thtemp(1) + thtemp(end));
                                else
                                    thtemp = seta3/180*pi : 0.01 : seta1/180*pi;
                                    
                                    curve_th = abs( thtemp(end) - thtemp(1) );
%                                     curve_th = abs( seta1 - seta3 );
                                end

                            else %왼쪽에 있네
                                if seta1 > 180
                                     tempangle1 = seta1/180*pi : 0.01 : 2*pi;
%                                      tempangle2 = 0 : 0.01 : seta3;
                                     tempangle2 = 0 : 0.01 : seta3/180*pi;
                                     thtemp = [tempangle1,tempangle2];
                                     
                                     curve_th = abs(2*pi-thtemp(1) + thtemp(end));
                                else
                                    thtemp = seta1/180*pi : 0.01 : seta3/180*pi;
                                    
                                    curve_th = abs( thtemp(end) - thtemp(1) );
%                                     curve_th = abs( seta1 - seta3 );
                                end
                            end
                        elseif y1 < y3 %시작점이 오아 y값이 크면 오쪽에있는거
                            if resulty > y2 %왼쪽에 있네
                                if seta1 < 180
                                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%??????????????????????????
                                    tempangle1 = 0 : 0.01 : seta1/180*pi;
                                    tempangle2 = seta3/180*pi : 0.01 : 2*pi;
                                    thtemp = [tempangle1,tempangle2];
                                    
                                    curve_th = abs(2*pi-tempangle2(1) + tempangle1(end));
%                                     curve_th = abs( seta1 + seta3 );
                                else
                                    thtemp = seta3/180*pi : 0.01 : seta1/180*pi;
                                    
                                    curve_th = abs( thtemp(end) - thtemp(1) );
%                                     curve_th = abs( seta1 - seta3 );
                                end
                            else %오쪽에 있네
                                if seta1 < 180
                                    thtemp = seta1/180*pi : 0.01 : seta3/180*pi;
                                    
                                    curve_th = abs( thtemp(end) - thtemp(1) );
%                                     curve_th = abs( seta1 - seta3 );
                                else
                                    tempangle1 = seta1/180*pi : 0.01 : (2*pi)-0.01;
                                    tempangle2 = 0 : 0.01 : seta3/180*pi;
                                    thtemp = [tempangle1,tempangle2];
                                    
                                    curve_th = abs(2*pi-tempangle1(1) + tempangle2(end));
%                                     curve_th = abs( seta1 - seta3 );
                                end
                            end
                        else
                            thtemp = seta3/180*pi : 0.01 : seta1/180*pi;
                            
                            curve_th = abs( thtemp(end) - thtemp(1) );
%                             curve_th = abs( seta1 - seta3 );
                        end
                    elseif x1 < x3 % 시작점이 왼쪽에 있을때
                        if y1 > y3 %시작점이 왼위 y값이 크면 오쪽에있는거
                            if resulty > y2 %왼쪽에 있네
                                if seta3 < 180
                                    tempangle1 = seta1/180*pi : 0.01 : 2*pi;
                                    tempangle2 = 0 : 0.01 : seta3/180*pi;
                                    thtemp = [tempangle1,tempangle2];
                                    
                                    curve_th = abs(2*pi-tempangle1(1) + tempangle2(end));
%                                     curve_th = abs( seta1 - seta3 );
                                else
                                    thtemp = seta1/180*pi : 0.01 : seta3/180*pi;
                                    
                                    curve_th = abs( thtemp(1) - thtemp(end) );
%                                     curve_th = abs( seta1 - seta3 );
                                end

                            else %오쪽에 있네
                                if seta3 <180
                                    thtemp = seta3/180*pi : 0.01 : seta1/180*pi;
                                    
                                    curve_th = abs( thtemp(end) - thtemp(1) );
%                                     curve_th = abs( seta1 - seta3 );
                                else
                                    tempangle1 = 0 : 0.01 : seta1/180*pi;
                                    tempangle2 = seta3/180*pi : 0.01 : 2*pi;
                                    thtemp = [tempangle2,tempangle1];
                                    
                                    curve_th = abs(tempangle1(end) + 2*pi-tempangle2(1));
%                                     curve_th = abs( seta1 - seta3 );
                                end
                            end
                        elseif y1 < y3 %시작점이 왼아 y값이 크면 왼쪽에있는거
                            if resulty > y2 %오른쪽에 있네
                                if seta3 < 180
                                    tempangle1 = seta1/180*pi : 0.01 : 2*pi;
                                    tempangle2 = 0 : 0.01 : seta3/180*pi;
                                    thtemp = [tempangle1,tempangle2];
                                    
                                    curve_th = abs(2*pi-tempangle1(1) + tempangle2(end));
%                                     curve_th = abs( seta1 - seta3 );
                                else
                                     thtemp = seta1/180*pi : 0.01 : seta3/180*pi;
                                     
                                     curve_th = abs( thtemp(end) - thtemp(1) );
%                                      curve_th = abs( seta1 - seta3 );
                                end

                            else %왼쪽에 있네
                                if seta3 < 180
                                    thtemp = seta3/180*pi : 0.01 : seta1/180*pi;
                                    
                                    curve_th = abs( thtemp(end) - thtemp(1) );
%                                     curve_th = abs( seta1 - seta3 );
                                else
                                    %%%%%%%%%%%%%%%%%%%%%%%%%       ?????????????????????????????????
                                    tempangle1 = 0 : 0.01 : seta1/180*pi;
                                    tempangle2 = seta3/180*pi : 0.01 : 2*pi;
                                    thtemp = [tempangle1,tempangle2];
                                    
                                    curve_th = abs(tempangle1(end) + 2*pi-tempangle2(1));
%                                     curve_th = abs( seta1 + seta3 );
                                end
                            end
                        else
                            thtemp = seta1/180*pi : 0.01 : seta3/180*pi;
                            
                            curve_th = abs( thtemp(end) - thtemp(1) );
%                             curve_th = abs( seta1 - seta3 );
                        end     

                    end
                    
                    angle = length(thtemp) * 0.57;
                    
                    % 호의 길이
                    curve_part_path = 2*pi*radius*(angle/360);
%                     curve_part_path = curve_th * radius;
                    
                end
                
                c_cxt = centerx; c_cyt = centery; c_rt =radius;
                xct = c_rt*cos(thtemp)+c_cxt; 
                yct = c_rt*sin(thtemp)+c_cyt;
                plot(xct,yct,'red','LineWidth',3);
                
                curve_part_length = curve_part_path + curve_part_length;
                curve_length(p) = curve_part_length;
                
                p=p+1;

            end
        end
        
        curve_path_all = curve_path_all + curve_part_length;
        bank_v_all{i} = bank_v_part;
        
    end
    
    assignin('base', 'thtemp', thtemp);
    assignin('base', 'curve_length', curve_length);
    assignin('base', 'banc_v_all', bank_v_all);
    set(handles.edit_drone_all_way, 'String', sprintf('%s [km]', num2str(curve_path_all)));
    
    drone_time = curve_path_all / d_velo;
    set(handles.edit_drone_time, 'String', sprintf('%s [h]', num2str(drone_time)));
    disp('전체 곡선 경로 길이');
    curve_path_all
        
    
end



function editSamsiteR_Callback(hObject, eventdata, handles)
% hObject    handle to editSamsiteR (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editSamsiteR as text
%        str2double(get(hObject,'String')) returns contents of editSamsiteR as a double


% --- Executes during object creation, after setting all properties.
function editSamsiteR_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editSamsiteR (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editHigh_Callback(hObject, eventdata, handles)
% hObject    handle to editHigh (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editHigh as text
%        str2double(get(hObject,'String')) returns contents of editHigh as a double


% --- Executes during object creation, after setting all properties.
function editHigh_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editHigh (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbuttonHigh.
function pushbuttonHigh_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonHigh (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% global high;

% 고도 입력 버튼 내용

global x_1d_s;
global y_1d_s;
global z_1d_s;
global x_2d_s;
global y_2d_s;
global z_2d_s;

global vx_new;
global vy_new;
global xn_new;
global yn_new;

global r_sq;

global vx1;
global vy1;
global vx2;
global vy2;

global N_s;

global buttonCount;

global xc_total;
global yc_total;
global mouseClickCount;

global vx_old;
global vy_old;

global samsiteX;
global samsiteY;
global samsiteR;



if(isempty(vx_old) ~= 0)
    
    for i = 1 : length(vx_old)
        
        vx_old = [];
        vy_old = [];
        
    end
    
end

% figure 내용 삭제
cla;
buttonCount = 0;



high = str2double(get(handles.editHigh, 'String'));

high

%2단계 위협표기
i_site = find(z_1d_s > high);  %높이 값주는부분 실제로 사용될 크기 판단
xn_new = zeros(1,length(i_site));  
yn_new = zeros(1,length(i_site));  
z_new = zeros(1,length(i_site));

for sitetmp = 1:length(i_site) % 높이에 의해서 걸러진 xy위치 저장
   xn_new(sitetmp) = x_1d_s(i_site(sitetmp));
   yn_new(sitetmp) = y_1d_s(i_site(sitetmp)); 
   z_new(sitetmp) = z_1d_s(i_site(sitetmp)); 
end

[vx, vy] = voronoi(xn_new,yn_new);
% [vx, vy] = voronoiComm(xn_new,yn_new);

% 1열의 데이터 획득
vx1 = vx(1,:); 
vy1 = vy(1,:);

% 2열의 데이터 획득
vx2 = vx(2,:);
vy2 = vy(2,:);

c_r = 1;
vx_new = vx;
vy_new = vy;

for i = 1 : length(xn_new)
    % 1열의 데이터 획득
    vx1 = vx_new(1,:); 
    vy1 = vy_new(1,:);

    % 2열의 데이터 획득
    vx2 = vx_new(2,:);
    vy2 = vy_new(2,:);

    r_sq = (vx1-xn_new(i)).^2+(vy1-yn_new(i)).^2;
    idx1 = find((r_sq < c_r^2));

    r_sq = (vx2-xn_new(i)).^2+(vy2-yn_new(i)).^2;
    idx2 = find((r_sq < c_r^2));

    idx = union(idx1,idx2);


    vx_new(:,idx) = [];
    vy_new(:,idx) = [];

end


% 샘사이트 위치를 지정한 정보가 있다면
% if((isempty(xc_total) ~= 0) && (isempty(yc_total) ~= 0))
if(mouseClickCount ~= 0)
    
    th = 0:0.01:2*pi; %원을 그리기위한 각도 배열
    
    for i = 1 : mouseClickCount
    
        c_cx = samsiteX(i); c_cy = samsiteY(i); c_r = samsiteR;

        xc = c_r*cos(th)+c_cx; 
        yc = c_r*sin(th)+c_cy;

        h = fill(xc,yc,'red');
        set(h,'facealpha',.5);

        vx1 = vx_new(1,:); 
        vy1 = vy_new(1,:);
        
        r_sq = (vx1-c_cx).^2+(vy1-c_cy).^2;
        idx1 = find((r_sq < c_r^2));

        vx2 = vx_new(2,:);
        vy2 = vy_new(2,:);

        r_sq = (vx2-c_cx).^2+(vy2-c_cy).^2;
        idx2 = find((r_sq < c_r^2));

        idx = union(idx1,idx2);


        vx_new(:,idx) = [];
        vy_new(:,idx) = [];
          
    
    end
    
end



%figure(1);
set(gcf,'numbertitle','off','name', '위협표기');
contour(x_2d_s,y_2d_s,z_2d_s,'ShowText','on');
xlabel('km');
ylabel('km');
hold on;
% plot(vx_new,vy_new,'b.-')
plot(xn_new,yn_new,'r.');
axis([0 N_s 0 N_s]);


% --- Executes on button press in pushbutton_drone_time.
function pushbutton_drone_time_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_drone_time (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function edit_drone_time_Callback(hObject, eventdata, handles)
% hObject    handle to edit_drone_time (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_drone_time as text
%        str2double(get(hObject,'String')) returns contents of edit_drone_time as a double


% --- Executes during object creation, after setting all properties.
function edit_drone_time_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_drone_time (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton_drone_velo.
function pushbutton_drone_velo_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_drone_velo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% 드론 속도 버튼 눌렀을 때 동작 내용

global d_velo;
global d_time;
% global d_s;

d_velo = str2double(get(handles.edit_drone_velocity, 'String'));
d_time = str2double(get(handles.edit_drone_time, 'String'));

d_time(1) = [];

assignin('base', 'd_time', d_time);
assignin('base', 'd_velo', d_velo);

drone_s = d_velo * d_time;

disp(drone_s);



function edit_drone_velocity_Callback(hObject, eventdata, handles)
% hObject    handle to edit_drone_velocity (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_drone_velocity as text
%        str2double(get(hObject,'String')) returns contents of edit_drone_velocity as a double


% --- Executes during object creation, after setting all properties.
function edit_drone_velocity_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_drone_velocity (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on key press with focus on edit_drone_velocity and none of its controls.
function edit_drone_velocity_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to edit_drone_velocity (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.CONTROL.UICONTROL)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)



function edit_drone_all_way_Callback(hObject, eventdata, handles)
% hObject    handle to edit_drone_all_way (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_drone_all_way as text
%        str2double(get(hObject,'String')) returns contents of edit_drone_all_way as a double


% --- Executes during object creation, after setting all properties.
function edit_drone_all_way_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_drone_all_way (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_drone_now_way_Callback(hObject, eventdata, handles)
% hObject    handle to edit_drone_now_way (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_drone_now_way as text
%        str2double(get(hObject,'String')) returns contents of edit_drone_now_way as a double


% --- Executes during object creation, after setting all properties.
function edit_drone_now_way_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_drone_now_way (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_drone_num_Callback(hObject, eventdata, handles)
% hObject    handle to edit_drone_num (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_drone_num as text
%        str2double(get(hObject,'String')) returns contents of edit_drone_num as a double


% --- Executes during object creation, after setting all properties.
function edit_drone_num_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_drone_num (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton_reset.
function pushbutton_reset_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_reset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% RESET

global r_sq;

global xc;
global yc;

global buttonCount;

global xc_total;
global yc_total;
global mouseClickCount;

global idx;

global vx_old;
global vy_old;

global vx1;
global vy1;
global vx2;
global vy2;

global samsiteX;
global samsiteY;
global samsiteR;

vx1 = [];
vy1 = [];
vx2 = [];
vy2 = [];

% vx_new = [];
% vy_new = [];
% xn_new = [];
% yn_new = [];

r_sq = [];

xc = [];
yc = [];
xc_total = xc;
yc_total = yc;

idx = [];

vx_old = [];
vy_old = [];

samsiteX = 0;
samsiteY = 0;
samsiteR = 0;

mouseClickCount = 0;

buttonCount = 0;

cla;

[Z, ref] = dted('n37.dt0');

N_s = 60;
x_s = linspace(0, N_s,N_s);
y_s = linspace(0, N_s,N_s);

x_2d_s = repmat(x_s,N_s,1);
y_2d_s = repmat(y_s',1,N_s);

x_1d_s = zeros(1,N_s*N_s);
y_1d_s = zeros(1,N_s*N_s);
z_1d_s = zeros(1,N_s*N_s);
z_2d_s = zeros(N_s,N_s);

cnt = 1;
for i = 1: N_s %1자로 만드는부분
    for j = 1: N_s
        x_1d_s(cnt) = x_2d_s(i,j);
        y_1d_s(cnt) = y_2d_s(i,j);
        z_1d_s(cnt) = Z(i,j);
        z_2d_s(i,j) = Z(i,j);
        cnt = cnt+1;
    end
end

%1단계 등고선지도 획득
%figure(1);
set(gcf,'numbertitle','off','name', '등고선지도');
contour(x_2d_s,y_2d_s,z_2d_s,'ShowText','on');
xlabel('km');
ylabel('km');



% --- Executes on button press in pushbutton_drone_v.
function pushbutton_drone_v_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_drone_v (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)





function edit_drone_v_Callback(hObject, eventdata, handles)
% hObject    handle to edit_drone_v (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_drone_v as text
%        str2double(get(hObject,'String')) returns contents of edit_drone_v as a double


% --- Executes during object creation, after setting all properties.
function edit_drone_v_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_drone_v (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
