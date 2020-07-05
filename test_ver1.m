clear;
clc;
cla;

[Z, ref] = dted('n37.dt0');

N_s = 70;
x_s = linspace(1, N_s,N_s);
y_s = linspace(1, N_s,N_s);

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

flight_height = 700;% 비행 높이보다 실제 높이가 높아 운항이 불가능한 곳(장애물)
% 장애물의 위치는 xn_new, yn_new, z_new 에 저장되어있음
i_site = find(z_1d_s > flight_height);  %높이 값주는부분 실제로 사용될 크기 판단
xn_new = zeros(1,length(i_site));  
yn_new = zeros(1,length(i_site));  
z_new = zeros(1,length(i_site));

for sitetmp = 1:length(i_site) % 높이에 의해서 걸러진 xy위치 저장
   xn_new(sitetmp) = x_1d_s(i_site(sitetmp));
   yn_new(sitetmp) = y_1d_s(i_site(sitetmp)); 
   z_new(sitetmp) = z_1d_s(i_site(sitetmp)); 
end

[vx, vy] = voronoi(xn_new,yn_new);

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

%4단계 샘사이트 경로 표기
th = 0:0.01:2*pi; %원을 그리기위한 각도 배열
c_cx = 15; c_cy = 17; c_r =5;
% c_cx = 25; c_cy = 85; c_r =5;
% c_cx = 0; c_cy = 0; c_r = 0;
xc = c_r*cos(th)+c_cx; 
yc = c_r*sin(th)+c_cy;

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

% 지면으로부터 비행을 위해 떨어진 거리(km)
height = 5;

% xpoint = [10;49;22;46;14;30];
% ypoint = [10;42;53;13;29;17];

%xpoint = [14; 10; 10];
%ypoint = [29; 55; 10];

% 논문용 1
% xpoint = [35; 83];
% ypoint = [95; 59];

% 논문용 2
% xpoint = [43; 100];
% ypoint = [105; 60];

% 논문용 3
% xpoint = [33; 75];
% ypoint = [102; 50];

% 논문용 4
% xpoint = [10; 90];
% ypoint = [90; 50];

% 논문용 5
% xpoint = [ 8; 18];
% ypoint = [50; 30];

% 논문용 6
xpoint = [30; 10];
ypoint = [60; 10];

% figure(5);
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
set(h,'facealpha',.5);
axis([0 N_s 0 N_s]);

% 선회 경로 총 길이
path_length_all = 0;

part_length = [];

% 최대 상승 높이(최대 운용 고도)
max_flight_height = 1000;

%% 추가 - 직선의 방정식 구하기

% 장애물 위치 파악
x_obs = [];
y_obs = [];

count_obs = 0;

% 직선의 방정식 : y = ax + b
res = polyfit(xpoint, ypoint, 1);
x_equation = xpoint(1) : (x_s(2)-x_s(1)) : xpoint(2);
y_equation = res(1)*x_equation + res(2);

%% 직선 경로 중에 있는 y 좌표 값 추출

% 상승 및 하강위한 높이값 추출 변수
z_lift_height = [];
% flight height를 반영한 높이 정보
lift_height = [];

y_index = [];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 직선 경로에 있는 점들중 일치하는 y 지점 찾기
for i = 1 : length(y_equation)

    flag = 0;

    for j = 2 : N_s-1
        num = find( y_equation(i) == y_s(j) );

        if(num)
            y_equation(i) = y_s(j);
            flag = 1;
            break;

        else
            flag = 0;

        end

    end

    if(flag == 0)
        y_equation(i) = y_s(knnsearch(y_s', y_equation(i)));

    end

    y_index(i) = knnsearch(y_s', y_equation(i));

end

%% 직선 경로의 z 좌표값 추출

lift_height = [];

obs_index = [];

% 장애물의 개수
count_obs = 0;

% 직선 경로에 있는 점들중 일치하는 z 지점 찾기
for k = 1 : length(y_equation)
    for i = 1 : N_s

        % x와 동일한 지점 2d에서 찾기
        if(find( x_equation(k) == x_s(i) ))
            z_lift_height(k) = z_2d_s( (y_index(k)-1)*N_s + i );

            if(z_lift_height(k) < flight_height)
                lift_height(k) = flight_height;

%             elseif(z_lift_height(k) == flight_height)
                count_obs = count_obs + 1;
%                 lift_height(k) = z_lift_height(k);
                obs_index(count_obs) = k;

            else
                lift_height(k) = z_lift_height(k);

            end

        end


    end

end

%%

th = [];

temp_away = 0;

x_up = [];
y_up = [];

x_down = [];
y_down = [];

% k == obs_index(i)
for i = 1 : count_obs

    k = obs_index(i);

    if( (k > temp_away) && (k ~= obs_index(end)) )

%         disp('yes');

        if( lift_height(k) < lift_height(k+1) )
%             disp('up');

            x_up(i) = x_equation(k);
            y_up(i) = y_equation(k);

            a = ( (lift_height(k+1) - lift_height(k-temp_away)) / (x_equation(k+1) - x_equation(k-temp_away)) );
            th_temp = atan(a);
            % radian -> degree
            th(i) = th_temp * 180 / pi;

%         elseif( lift_height(k) < lift_height(k-1) )
%             x_down(i) = x_equation(k);
%             y_down(i) = y_equation(k);
            
    %         disp('down');

        else
%             disp('nothing');

        end

    end

end

%% 1 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x_equation_1 = x_equation(1:end);
y_equation_1 = y_equation(1:end);
lift_height_1 = lift_height(1:end);

% lift_height_1 = [

% 상승 및 하강 경로 총 길이
lift_length = 0;
%z_lift_height
lift_length_part = [];
%lift_heigth

for i = 1 : length(y_equation_1)-1
    lift_length_part(i) = sqrt( (x_equation_1(i+1)-x_equation_1(i))^2 + (y_equation_1(i+1)-y_equation_1(i))^2 + ( (lift_height_1(i+1)-lift_height_1(i))/1000 )^2 );

    lift_length = lift_length + sqrt( (x_equation_1(i+1)-x_equation_1(i))^2 + (y_equation_1(i+1)-y_equation_1(i))^2 + ( (lift_height_1(i+1)-lift_height_1(i))/1000 )^2 );

end

xpoint = [x_equation_1(1) x_equation_1(end)];
ypoint = [y_equation_1(1) y_equation_1(end)];

order = [1 2];

for mainloof = 1 : length(order)-1

    xy_start = [xpoint(order(mainloof)) ypoint(order(mainloof))];
    xy_dest = [xpoint(order(mainloof+1)) ypoint(order(mainloof+1))];
    
    %상승 및 하강
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % 추가 
    
    x_obs_pos = []; 
    y_obs_pos = [];
    
    x_obs_location = [];
    y_obs_location = [];
    
    obs_pos = [];
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % 장애물 위치 파악
    x_obs = [];
    y_obs = [];
    
    count_obs = 0;
    
    for i = 1 : length(x_equation)
        
        x_obs_pos = find(x_equation(i) == xn_new);
        y_obs_pos = find(y_equation(i) == yn_new);
        
    end
    
    for i = 1 : length(x_obs_pos)
        obs_pos = find(x_obs_pos(i) == y_obs_pos);
        %obs_pos
        
        if(obs_pos)
            x_obs_location(i) = xn_new(x_obs_pos(obs_pos));
            y_obs_location(i) = yn_new(y_obs_pos(obs_pos));
            
        end
         
    end
    
    % 여기 까지
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
    
    figure(1);
    plot(xy_start(1),xy_start(2),'S',xy_dest(1),xy_dest(2),'D','MarkerSize',10, 'MarkerFaceColor','red');
    plot(xy_opt_path(:,1),xy_opt_path(:,2),'LineWidth',4);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % z 축의 데이터 도출
    
    zd = [];
    z_data = [];
    
    for i = 1 : length(xd)
        
        pos_x = 10000;
        pos_y = 10000;
        
        x_data = 0;
        y_data = 0;
        
        % 가장 가까운 위치로 지정
        for j = 1 : length(x_s)
           
            x_length = abs( x_s(j) - xd(i) );
            y_length = abs( y_s(j) - yd(i) );
            
            if( pos_x > x_length )
                pos_x = x_length;
                
                x_data = x_s(j);
            else
                pos_x = pos_x;
                x_data = x_data;
            end
            
            if( pos_y > y_length )
                pos_y = y_length;
                y_data = y_s(j);
            else
                pos_y = pos_y;
                y_data = y_data;
            end
            
        end
        
        % 해당 위치의 x좌표의 값과 y좌표의 값을 찾아 저장
        temp_pos_x = find( x_data == x_2d_s );
        temp_pos_y = find( y_data == y_2d_s );
        temp_pos_z = 0;
        
        for k = 1 : length(temp_pos_x)
            temp = find( temp_pos_x(k) == temp_pos_y );
            
            if( temp )
                temp_pos_z = temp;
                
                z_data(i) = z_2d_s(temp_pos_z);
                
                % 비행고도 제한 계산
                if( z_data(i) < flight_height )
                    z_data(i) = flight_height + height;
                end
                
            else
                temp_pos_z = temp_pos_z;
                
            end
            
        end
        
    end
    
    % 선회 직선 경로 데이터
    ttemp_xy_path_all{mainloof} = xy_opt_path;

    rootCount = 0;

    % 전체 경로 길이 구간별 계산
    path1 = 0;

    for i = 1 : length(xy_opt_path)-1

        path_part = sqrt((xy_opt_path(i,1) - xy_opt_path(i+1,1))^2 + (xy_opt_path(i,2) - xy_opt_path(i+1,2))^2 + ( (z_data(i+1)-z_data(i)) /1000)^2);
        path1 = path_part + path1;
        part_length(i) = path_part;

    end

    path_length(mainloof) = path1;
    part_length_all{mainloof} = part_length;

%     path_length_all = path_length(mainloof) + path_length_all;
    path_length_all = path1 + path_length_all;
    
        
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    
end

disp('전체 상승 경로 길이');
    lift_length

disp('전체 선회 경로 길이');
    path_length_all

%%

x_all = [];
y_all = [];
z_all = [];

length_all = 0;

if(path_length_all > lift_length)
    x_all = [x_all x_equation_1];
    y_all = [y_all y_equation_1];
    z_all = [z_all lift_height_1];
    
%     disp('전체 상승 경로 길이');
%     lift_length
    
    length_all = length_all + lift_length

else
    x_all = [x_all xy_opt_path(:,1)];
    y_all = [y_all xy_opt_path(:,2)];
    
    z_temp = [];
    for i = 1 : length(lift_height_1)
        z_temp(i) = flight_height;
        
    end
    
    z_all = [z_all z_temp];
    
    disp('전체 선회 경로 길이');
    path_length_all
    
    length_all = length_all + path_length_all
    
end

figure(2)
% making 3D MAP
meshc(x_2d_s, y_2d_s, z_2d_s);
hold on;

% using plot3 function   ex. plot3(x1, y1, z1, ...)
plot3(x_all, y_all, z_all + 10, 'red', 'LineWidth',4);

%% 2 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x_equation_1 = x_equation(5:12);
y_equation_1 = y_equation(5:12);
lift_height_1 = lift_height(5:12);

% lift_length = 1000000000000000000000;
% 상승 및 하강 경로 총 길이
lift_length = 0;
%z_lift_height
lift_length_part = [];
%lift_heigth

for i = 1 : length(y_equation_1)-1
    lift_length_part(i) = sqrt( (x_equation_1(i+1)-x_equation_1(i))^2 + (y_equation_1(i+1)-y_equation_1(i))^2 + ( (lift_height_1(i+1)-lift_height_1(i))/1000 )^2 );

    lift_length = lift_length + sqrt( (x_equation_1(i+1)-x_equation_1(i))^2 + (y_equation_1(i+1)-y_equation_1(i))^2 + ( (lift_height_1(i+1)-lift_height_1(i))/1000 )^2 );

end

disp('전체 상승 경로 길이');
lift_length

xpoint = [x_equation_1(1) x_equation_1(end)];
ypoint = [y_equation_1(1) y_equation_1(end)];

order = [1 2];

for mainloof = 1 : length(order)-1

    xy_start = [xpoint(order(mainloof)) ypoint(order(mainloof))];
    xy_dest = [xpoint(order(mainloof+1)) ypoint(order(mainloof+1))];
    
    %상승 및 하강
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % 추가 
    
    x_obs_pos = []; 
    y_obs_pos = [];
    
    x_obs_location = [];
    y_obs_location = [];
    
    obs_pos = [];
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % 장애물 위치 파악
    x_obs = [];
    y_obs = [];
    
    count_obs = 0;
    
    for i = 1 : length(x_equation)
        
        x_obs_pos = find(x_equation(i) == xn_new);
        y_obs_pos = find(y_equation(i) == yn_new);
        
    end
    
    for i = 1 : length(x_obs_pos)
        obs_pos = find(x_obs_pos(i) == y_obs_pos);
        %obs_pos
        
        if(obs_pos)
            x_obs_location(i) = xn_new(x_obs_pos(obs_pos));
            y_obs_location(i) = yn_new(y_obs_pos(obs_pos));
            
        end
         
    end
    
    % 여기 까지
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
    
    figure(1);
    plot(xy_start(1),xy_start(2),'S',xy_dest(1),xy_dest(2),'D','MarkerSize',10, 'MarkerFaceColor','red');
    plot(xy_opt_path(:,1),xy_opt_path(:,2),'LineWidth',4);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % z 축의 데이터 도출
    
    zd = [];
    z_data = [];
    
    for i = 1 : length(xd)
        
        pos_x = 10000;
        pos_y = 10000;
        
        x_data = 0;
        y_data = 0;
        
        % 가장 가까운 위치로 지정
        for j = 1 : length(x_s)
           
            x_length = abs( x_s(j) - xd(i) );
            y_length = abs( y_s(j) - yd(i) );
            
            if( pos_x > x_length )
                pos_x = x_length;
                
                x_data = x_s(j);
            else
                pos_x = pos_x;
                x_data = x_data;
            end
            
            if( pos_y > y_length )
                pos_y = y_length;
                y_data = y_s(j);
            else
                pos_y = pos_y;
                y_data = y_data;
            end
            
        end
        
        % 해당 위치의 x좌표의 값과 y좌표의 값을 찾아 저장
        temp_pos_x = find( x_data == x_2d_s );
        temp_pos_y = find( y_data == y_2d_s );
        temp_pos_z = 0;
        
        for k = 1 : length(temp_pos_x)
            temp = find( temp_pos_x(k) == temp_pos_y );
            
            if( temp )
                temp_pos_z = temp;
                
                z_data(i) = z_2d_s(temp_pos_z);
                
                % 비행고도 제한 계산
                if( z_data(i) < flight_height )
                    z_data(i) = flight_height + height;
                end
                
            else
                temp_pos_z = temp_pos_z;
                
            end
            
        end
        
    end
    
    % 선회 직선 경로 데이터
    ttemp_xy_path_all{mainloof} = xy_opt_path;

    rootCount = 0;

    % 전체 경로 길이 구간별 계산
    path1 = 0;

    for i = 1 : length(xy_opt_path)-1

        path_part = sqrt((xy_opt_path(i,1) - xy_opt_path(i+1,1))^2 + (xy_opt_path(i,2) - xy_opt_path(i+1,2))^2 + ( (z_data(i+1)-z_data(i)) /1000)^2);
        path1 = path_part + path1;
        part_length(i) = path_part;

    end

    path_length(mainloof) = path1;
    part_length_all{mainloof} = part_length;

%     path_length_all = path_length(mainloof) + path_length_all;
    path_length_all = path1 + path_length_all;
    
        
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    
end

%%

if(path_length_all > lift_length)
    x_all = [x_all x_equation_1];
    y_all = [y_all y_equation_1];
    z_all = [z_all lift_height_1];
    
    disp('전체 상승 경로 길이');
    lift_length
    
    length_all = length_all + lift_length

else
    x_all = [x_all xy_opt_path(:,1)'];
    y_all = [y_all xy_opt_path(:,2)'];
    
    z_temp = [];
    for i = 1 : length(xy_opt_path(:,1))
        z_temp(i) = flight_height;
        
    end
    
    z_all = [z_all z_temp];
    
    disp('전체 선회 경로 길이');
    path_length_all
    
    length_all = length_all + path_length_all
    
end

%% 3 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x_equation_1 = x_equation(12:15);
y_equation_1 = y_equation(12:15);
lift_height_1 = lift_height(12:15);

% 상승 및 하강 경로 총 길이
lift_length = 0;
%z_lift_height
lift_length_part = [];
%lift_heigth

for i = 1 : length(y_equation_1)-1
    lift_length_part(i) = sqrt( (x_equation_1(i+1)-x_equation_1(i))^2 + (y_equation_1(i+1)-y_equation_1(i))^2 + ( (lift_height_1(i+1)-lift_height_1(i))/1000 )^2 );

    lift_length = lift_length + sqrt( (x_equation_1(i+1)-x_equation_1(i))^2 + (y_equation_1(i+1)-y_equation_1(i))^2 + ( (lift_height_1(i+1)-lift_height_1(i))/1000 )^2 );

end

xpoint = [x_equation_1(1) x_equation_1(end)];
ypoint = [y_equation_1(1) y_equation_1(end)];

order = [1 2];

for mainloof = 1 : length(order)-1

    xy_start = [xpoint(order(mainloof)) ypoint(order(mainloof))];
    xy_dest = [xpoint(order(mainloof+1)) ypoint(order(mainloof+1))];
    
    %상승 및 하강
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % 추가 
    
    x_obs_pos = []; 
    y_obs_pos = [];
    
    x_obs_location = [];
    y_obs_location = [];
    
    obs_pos = [];
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % 장애물 위치 파악
    x_obs = [];
    y_obs = [];
    
    count_obs = 0;
    
    for i = 1 : length(x_equation)
        
        x_obs_pos = find(x_equation(i) == xn_new);
        y_obs_pos = find(y_equation(i) == yn_new);
        
    end
    
    for i = 1 : length(x_obs_pos)
        obs_pos = find(x_obs_pos(i) == y_obs_pos);
        %obs_pos
        
        if(obs_pos)
            x_obs_location(i) = xn_new(x_obs_pos(obs_pos));
            y_obs_location(i) = yn_new(y_obs_pos(obs_pos));
            
        end
         
    end
    
    % 여기 까지
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
    
    figure(1);
    plot(xy_start(1),xy_start(2),'S',xy_dest(1),xy_dest(2),'D','MarkerSize',10, 'MarkerFaceColor','red');
    plot(xy_opt_path(:,1),xy_opt_path(:,2),'LineWidth',4);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % z 축의 데이터 도출
    
    zd = [];
    z_data = [];
    
    for i = 1 : length(xd)
        
        pos_x = 10000;
        pos_y = 10000;
        
        x_data = 0;
        y_data = 0;
        
        % 가장 가까운 위치로 지정
        for j = 1 : length(x_s)
           
            x_length = abs( x_s(j) - xd(i) );
            y_length = abs( y_s(j) - yd(i) );
            
            if( pos_x > x_length )
                pos_x = x_length;
                
                x_data = x_s(j);
            else
                pos_x = pos_x;
                x_data = x_data;
            end
            
            if( pos_y > y_length )
                pos_y = y_length;
                y_data = y_s(j);
            else
                pos_y = pos_y;
                y_data = y_data;
            end
            
        end
        
        % 해당 위치의 x좌표의 값과 y좌표의 값을 찾아 저장
        temp_pos_x = find( x_data == x_2d_s );
        temp_pos_y = find( y_data == y_2d_s );
        temp_pos_z = 0;
        
        for k = 1 : length(temp_pos_x)
            temp = find( temp_pos_x(k) == temp_pos_y );
            
            if( temp )
                temp_pos_z = temp;
                
                z_data(i) = z_2d_s(temp_pos_z);
                
                % 비행고도 제한 계산
                if( z_data(i) < flight_height )
                    z_data(i) = flight_height + height;
                end
                
            else
                temp_pos_z = temp_pos_z;
                
            end
            
        end
        
    end
    
    % 선회 직선 경로 데이터
    ttemp_xy_path_all{mainloof} = xy_opt_path;

    rootCount = 0;

    % 전체 경로 길이 구간별 계산
    path1 = 0;

    for i = 1 : length(xy_opt_path)-1

        path_part = sqrt((xy_opt_path(i,1) - xy_opt_path(i+1,1))^2 + (xy_opt_path(i,2) - xy_opt_path(i+1,2))^2 + ( (z_data(i+1)-z_data(i)) /1000)^2);
        path1 = path_part + path1;
        part_length(i) = path_part;

    end

    path_length(mainloof) = path1;
    part_length_all{mainloof} = part_length;

%     path_length_all = path_length(mainloof) + path_length_all;
    path_length_all = path1 + path_length_all;
    
        
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    
end

%%

if(path_length_all > lift_length)
    x_all = [x_all x_equation_1];
    y_all = [y_all y_equation_1];
    z_all = [z_all lift_height_1];
    
    disp('전체 상승 경로 길이');
    lift_length
    
    length_all = length_all + lift_length

else
    x_all = [x_all xy_opt_path(:,1)'];
    y_all = [y_all xy_opt_path(:,2)'];
    
    z_temp = [];
    for i = 1 : length(xy_opt_path(:,1))
        z_temp(i) = flight_height;
        
    end
    
    z_all = [z_all z_temp];
    
    disp('전체 선회 경로 길이');
    path_length_all
    
    length_all = length_all + path_length_all
    
end

%% 4 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x_equation_1 = x_equation(15:17);
y_equation_1 = y_equation(15:17);
lift_height_1 = lift_height(15:17);

% 상승 및 하강 경로 총 길이
lift_length = 0;
%z_lift_height
lift_length_part = [];
%lift_heigth

for i = 1 : length(y_equation_1)-1
    lift_length_part(i) = sqrt( (x_equation_1(i+1)-x_equation_1(i))^2 + (y_equation_1(i+1)-y_equation_1(i))^2 + ( (lift_height_1(i+1)-lift_height_1(i))/1000 )^2 );

    lift_length = lift_length + sqrt( (x_equation_1(i+1)-x_equation_1(i))^2 + (y_equation_1(i+1)-y_equation_1(i))^2 + ( (lift_height_1(i+1)-lift_height_1(i))/1000 )^2 );

end

xpoint = [x_equation_1(1) x_equation_1(end)];
ypoint = [y_equation_1(1) y_equation_1(end)];

order = [1 2];

for mainloof = 1 : length(order)-1

    xy_start = [xpoint(order(mainloof)) ypoint(order(mainloof))];
    xy_dest = [xpoint(order(mainloof+1)) ypoint(order(mainloof+1))];
    
    %상승 및 하강
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % 추가 
    
    x_obs_pos = []; 
    y_obs_pos = [];
    
    x_obs_location = [];
    y_obs_location = [];
    
    obs_pos = [];
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % 장애물 위치 파악
    x_obs = [];
    y_obs = [];
    
    count_obs = 0;
    
    for i = 1 : length(x_equation)
        
        x_obs_pos = find(x_equation(i) == xn_new);
        y_obs_pos = find(y_equation(i) == yn_new);
        
    end
    
    for i = 1 : length(x_obs_pos)
        obs_pos = find(x_obs_pos(i) == y_obs_pos);
        %obs_pos
        
        if(obs_pos)
            x_obs_location(i) = xn_new(x_obs_pos(obs_pos));
            y_obs_location(i) = yn_new(y_obs_pos(obs_pos));
            
        end
         
    end
    
    % 여기 까지
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
    
    figure(1);
    plot(xy_start(1),xy_start(2),'S',xy_dest(1),xy_dest(2),'D','MarkerSize',10, 'MarkerFaceColor','red');
    plot(xy_opt_path(:,1),xy_opt_path(:,2),'LineWidth',4);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % z 축의 데이터 도출
    
    zd = [];
    z_data = [];
    
    for i = 1 : length(xd)
        
        pos_x = 10000;
        pos_y = 10000;
        
        x_data = 0;
        y_data = 0;
        
        % 가장 가까운 위치로 지정
        for j = 1 : length(x_s)
           
            x_length = abs( x_s(j) - xd(i) );
            y_length = abs( y_s(j) - yd(i) );
            
            if( pos_x > x_length )
                pos_x = x_length;
                
                x_data = x_s(j);
            else
                pos_x = pos_x;
                x_data = x_data;
            end
            
            if( pos_y > y_length )
                pos_y = y_length;
                y_data = y_s(j);
            else
                pos_y = pos_y;
                y_data = y_data;
            end
            
        end
        
        % 해당 위치의 x좌표의 값과 y좌표의 값을 찾아 저장
        temp_pos_x = find( x_data == x_2d_s );
        temp_pos_y = find( y_data == y_2d_s );
        temp_pos_z = 0;
        
        for k = 1 : length(temp_pos_x)
            temp = find( temp_pos_x(k) == temp_pos_y );
            
            if( temp )
                temp_pos_z = temp;
                
                z_data(i) = z_2d_s(temp_pos_z);
                
                % 비행고도 제한 계산
                if( z_data(i) < flight_height )
                    z_data(i) = flight_height + height;
                end
                
            else
                temp_pos_z = temp_pos_z;
                
            end
            
        end
        
    end
    
    % 선회 직선 경로 데이터
    ttemp_xy_path_all{mainloof} = xy_opt_path;

    rootCount = 0;

    % 전체 경로 길이 구간별 계산
    path1 = 0;

    for i = 1 : length(xy_opt_path)-1

        path_part = sqrt((xy_opt_path(i,1) - xy_opt_path(i+1,1))^2 + (xy_opt_path(i,2) - xy_opt_path(i+1,2))^2 + ( (z_data(i+1)-z_data(i)) /1000)^2);
        path1 = path_part + path1;
        part_length(i) = path_part;

    end

    path_length(mainloof) = path1;
    part_length_all{mainloof} = part_length;

%     path_length_all = path_length(mainloof) + path_length_all;
    path_length_all = path1 + path_length_all;
    
        
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    
end

%%

if(path_length_all > lift_length)
    x_all = [x_all x_equation_1];
    y_all = [y_all y_equation_1];
    z_all = [z_all lift_height_1];
    
    disp('전체 상승 경로 길이');
    lift_length
    
    length_all = length_all + lift_length

else
    x_all = [x_all xy_opt_path(:,1)'];
    y_all = [y_all xy_opt_path(:,2)'];
    
    z_temp = [];
    for i = 1 : length(xy_opt_path(:,1))
        z_temp(i) = flight_height;
        
    end
    
    z_all = [z_all z_temp];
    
    disp('전체 선회 경로 길이');
    path_length_all
    
    length_all = length_all + path_length_all
    
end

%% 5 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x_equation_1 = x_equation(17:24);
y_equation_1 = y_equation(17:24);
lift_height_1 = lift_height(17:24);

lift_length = 100000000000000000000000;
% % 상승 및 하강 경로 총 길이
% lift_length = 0;
% %z_lift_height
% lift_length_part = [];
% %lift_heigth
% 
% for i = 1 : length(y_equation_1)-1
%     lift_length_part(i) = sqrt( (x_equation_1(i+1)-x_equation_1(i))^2 + (y_equation_1(i+1)-y_equation_1(i))^2 + ( (lift_height_1(i+1)-lift_height_1(i))/1000 )^2 );
% 
%     lift_length = lift_length + sqrt( (x_equation_1(i+1)-x_equation_1(i))^2 + (y_equation_1(i+1)-y_equation_1(i))^2 + ( (lift_height_1(i+1)-lift_height_1(i))/1000 )^2 );
% 
% end
% 
% disp('전체 상승 경로 길이');
% lift_length

xpoint = [x_equation_1(1) x_equation_1(end)];
ypoint = [y_equation_1(1) y_equation_1(end)];

order = [1 2];

for mainloof = 1 : length(order)-1

    xy_start = [xpoint(order(mainloof)) ypoint(order(mainloof))];
    xy_dest = [xpoint(order(mainloof+1)) ypoint(order(mainloof+1))];
    
    %상승 및 하강
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % 추가 
    
    x_obs_pos = []; 
    y_obs_pos = [];
    
    x_obs_location = [];
    y_obs_location = [];
    
    obs_pos = [];
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % 장애물 위치 파악
    x_obs = [];
    y_obs = [];
    
    count_obs = 0;
    
    for i = 1 : length(x_equation)
        
        x_obs_pos = find(x_equation(i) == xn_new);
        y_obs_pos = find(y_equation(i) == yn_new);
        
    end
    
    for i = 1 : length(x_obs_pos)
        obs_pos = find(x_obs_pos(i) == y_obs_pos);
        %obs_pos
        
        if(obs_pos)
            x_obs_location(i) = xn_new(x_obs_pos(obs_pos));
            y_obs_location(i) = yn_new(y_obs_pos(obs_pos));
            
        end
         
    end
    
    % 여기 까지
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
    
    figure(1);
    plot(xy_start(1),xy_start(2),'S',xy_dest(1),xy_dest(2),'D','MarkerSize',10, 'MarkerFaceColor','red');
    plot(xy_opt_path(:,1),xy_opt_path(:,2),'LineWidth',4);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % z 축의 데이터 도출
    
    zd = [];
    z_data = [];
    
    for i = 1 : length(xd)
        
        pos_x = 10000;
        pos_y = 10000;
        
        x_data = 0;
        y_data = 0;
        
        % 가장 가까운 위치로 지정
        for j = 1 : length(x_s)
           
            x_length = abs( x_s(j) - xd(i) );
            y_length = abs( y_s(j) - yd(i) );
            
            if( pos_x > x_length )
                pos_x = x_length;
                
                x_data = x_s(j);
            else
                pos_x = pos_x;
                x_data = x_data;
            end
            
            if( pos_y > y_length )
                pos_y = y_length;
                y_data = y_s(j);
            else
                pos_y = pos_y;
                y_data = y_data;
            end
            
        end
        
        % 해당 위치의 x좌표의 값과 y좌표의 값을 찾아 저장
        temp_pos_x = find( x_data == x_2d_s );
        temp_pos_y = find( y_data == y_2d_s );
        temp_pos_z = 0;
        
        for k = 1 : length(temp_pos_x)
            temp = find( temp_pos_x(k) == temp_pos_y );
            
            if( temp )
                temp_pos_z = temp;
                
                z_data(i) = z_2d_s(temp_pos_z);
                
                % 비행고도 제한 계산
                if( z_data(i) < flight_height )
                    z_data(i) = flight_height + height;
                end
                
            else
                temp_pos_z = temp_pos_z;
                
            end
            
        end
        
    end
    
    % 선회 직선 경로 데이터
    ttemp_xy_path_all{mainloof} = xy_opt_path;

    rootCount = 0;

    % 전체 경로 길이 구간별 계산
    path1 = 0;

    for i = 1 : length(xy_opt_path)-1

        path_part = sqrt((xy_opt_path(i,1) - xy_opt_path(i+1,1))^2 + (xy_opt_path(i,2) - xy_opt_path(i+1,2))^2 + ( (z_data(i+1)-z_data(i)) /1000)^2);
        path1 = path_part + path1;
        part_length(i) = path_part;

    end

    path_length(mainloof) = path1;
    part_length_all{mainloof} = part_length;

%     path_length_all = path_length(mainloof) + path_length_all;
    path_length_all = path1 + path_length_all;
    
        
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    
end

%%

if(path_length_all > lift_length)
    x_all = [x_all x_equation_1];
    y_all = [y_all y_equation_1];
    z_all = [z_all lift_height_1];
    
    disp('전체 상승 경로 길이');
    lift_length
    
    length_all = length_all + lift_length

else
    x_all = [x_all xy_opt_path(:,1)'];
    y_all = [y_all xy_opt_path(:,2)'];
    
    z_temp = [];
    for i = 1 : length(xy_opt_path(:,1))
        z_temp(i) = flight_height;
        
    end
    
    z_all = [z_all z_temp];
    
    disp('전체 선회 경로 길이');
    path_length_all
    
    length_all = length_all + path_length_all
    
end

%% 6 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x_equation_1 = x_equation(24:26);
y_equation_1 = y_equation(24:26);
lift_height_1 = lift_height(24:26);

% 상승 및 하강 경로 총 길이
lift_length = 0;
%z_lift_height
lift_length_part = [];
%lift_heigth

for i = 1 : length(y_equation_1)-1
    lift_length_part(i) = sqrt( (x_equation_1(i+1)-x_equation_1(i))^2 + (y_equation_1(i+1)-y_equation_1(i))^2 + ( (lift_height_1(i+1)-lift_height_1(i))/1000 )^2 );

    lift_length = lift_length + sqrt( (x_equation_1(i+1)-x_equation_1(i))^2 + (y_equation_1(i+1)-y_equation_1(i))^2 + ( (lift_height_1(i+1)-lift_height_1(i))/1000 )^2 );

end

xpoint = [x_equation_1(1) x_equation_1(end)];
ypoint = [y_equation_1(1) y_equation_1(end)];

order = [1 2];

for mainloof = 1 : length(order)-1

    xy_start = [xpoint(order(mainloof)) ypoint(order(mainloof))];
    xy_dest = [xpoint(order(mainloof+1)) ypoint(order(mainloof+1))];
    
    %상승 및 하강
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % 추가 
    
    x_obs_pos = []; 
    y_obs_pos = [];
    
    x_obs_location = [];
    y_obs_location = [];
    
    obs_pos = [];
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % 장애물 위치 파악
    x_obs = [];
    y_obs = [];
    
    count_obs = 0;
    
    for i = 1 : length(x_equation)
        
        x_obs_pos = find(x_equation(i) == xn_new);
        y_obs_pos = find(y_equation(i) == yn_new);
        
    end
    
    for i = 1 : length(x_obs_pos)
        obs_pos = find(x_obs_pos(i) == y_obs_pos);
        %obs_pos
        
        if(obs_pos)
            x_obs_location(i) = xn_new(x_obs_pos(obs_pos));
            y_obs_location(i) = yn_new(y_obs_pos(obs_pos));
            
        end
         
    end
    
    % 여기 까지
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
    
    figure(1);
    plot(xy_start(1),xy_start(2),'S',xy_dest(1),xy_dest(2),'D','MarkerSize',10, 'MarkerFaceColor','red');
    plot(xy_opt_path(:,1),xy_opt_path(:,2),'LineWidth',4);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % z 축의 데이터 도출
    
    zd = [];
    z_data = [];
    
    for i = 1 : length(xd)
        
        pos_x = 10000;
        pos_y = 10000;
        
        x_data = 0;
        y_data = 0;
        
        % 가장 가까운 위치로 지정
        for j = 1 : length(x_s)
           
            x_length = abs( x_s(j) - xd(i) );
            y_length = abs( y_s(j) - yd(i) );
            
            if( pos_x > x_length )
                pos_x = x_length;
                
                x_data = x_s(j);
            else
                pos_x = pos_x;
                x_data = x_data;
            end
            
            if( pos_y > y_length )
                pos_y = y_length;
                y_data = y_s(j);
            else
                pos_y = pos_y;
                y_data = y_data;
            end
            
        end
        
        % 해당 위치의 x좌표의 값과 y좌표의 값을 찾아 저장
        temp_pos_x = find( x_data == x_2d_s );
        temp_pos_y = find( y_data == y_2d_s );
        temp_pos_z = 0;
        
        for k = 1 : length(temp_pos_x)
            temp = find( temp_pos_x(k) == temp_pos_y );
            
            if( temp )
                temp_pos_z = temp;
                
                z_data(i) = z_2d_s(temp_pos_z);
                
                % 비행고도 제한 계산
                if( z_data(i) < flight_height )
                    z_data(i) = flight_height + height;
                end
                
            else
                temp_pos_z = temp_pos_z;
                
            end
            
        end
        
    end
    
    % 선회 직선 경로 데이터
    ttemp_xy_path_all{mainloof} = xy_opt_path;

    rootCount = 0;

    % 전체 경로 길이 구간별 계산
    path1 = 0;

    for i = 1 : length(xy_opt_path)-1

        path_part = sqrt((xy_opt_path(i,1) - xy_opt_path(i+1,1))^2 + (xy_opt_path(i,2) - xy_opt_path(i+1,2))^2 + ( (z_data(i+1)-z_data(i)) /1000)^2);
        path1 = path_part + path1;
        part_length(i) = path_part;

    end

    path_length(mainloof) = path1;
    part_length_all{mainloof} = part_length;

%     path_length_all = path_length(mainloof) + path_length_all;
    path_length_all = path1 + path_length_all;
    
        
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    
end

%%

if(path_length_all > lift_length)
    x_all = [x_all x_equation_1];
    y_all = [y_all y_equation_1];
    z_all = [z_all lift_height_1];
    
    disp('전체 상승 경로 길이');
    lift_length
    
    length_all = length_all + lift_length

else
    x_all = [x_all xy_opt_path(:,1)'];
    y_all = [y_all xy_opt_path(:,2)'];
    
    z_temp = [];
    for i = 1 : length(xy_opt_path(:,1))
        z_temp(i) = flight_height;
        
    end
    
    z_all = [z_all z_temp];
    
    disp('전체 선회 경로 길이');
    path_length_all
    
    length_all = length_all + path_length_all
    
end

%% 7 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x_equation_1 = x_equation(26:39);
y_equation_1 = y_equation(26:39);
lift_height_1 = lift_height(26:39);

% for j = 1 : length(lift_height_1)
%     if(lift_height_1(j) < flight_height)
%         lift_height_1(j) = 1200;
%     end
% end

% lift_length = 1000000000000000000000000000;
% 상승 및 하강 경로 총 길이
lift_length = 0;
lift_length_part = [];

for i = 1 : length(y_equation_1)-1
    lift_length_part(i) = sqrt( (x_equation_1(i+1)-x_equation_1(i))^2 + (y_equation_1(i+1)-y_equation_1(i))^2 + ( (lift_height_1(i+1)-lift_height_1(i))/1000 )^2 );

    lift_length = lift_length + sqrt( (x_equation_1(i+1)-x_equation_1(i))^2 + (y_equation_1(i+1)-y_equation_1(i))^2 + ( (lift_height_1(i+1)-lift_height_1(i))/1000 )^2 );

end

xpoint = [x_equation_1(1) x_equation_1(end)];
ypoint = [y_equation_1(1) y_equation_1(end)];

order = [1 2];

for mainloof = 1 : length(order)-1

    xy_start = [xpoint(order(mainloof)) ypoint(order(mainloof))];
    xy_dest = [xpoint(order(mainloof+1)) ypoint(order(mainloof+1))];
    
    %상승 및 하강
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % 추가 
    
    x_obs_pos = []; 
    y_obs_pos = [];
    
    x_obs_location = [];
    y_obs_location = [];
    
    obs_pos = [];
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % 장애물 위치 파악
    x_obs = [];
    y_obs = [];
    
    count_obs = 0;
    
    for i = 1 : length(x_equation)
        
        x_obs_pos = find(x_equation(i) == xn_new);
        y_obs_pos = find(y_equation(i) == yn_new);
        
    end
    
    for i = 1 : length(x_obs_pos)
        obs_pos = find(x_obs_pos(i) == y_obs_pos);
        %obs_pos
        
        if(obs_pos)
            x_obs_location(i) = xn_new(x_obs_pos(obs_pos));
            y_obs_location(i) = yn_new(y_obs_pos(obs_pos));
            
        end
         
    end
    
    % 여기 까지
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
    
    figure(1);
    plot(xy_start(1),xy_start(2),'S',xy_dest(1),xy_dest(2),'D','MarkerSize',10, 'MarkerFaceColor','red');
    plot(xy_opt_path(:,1),xy_opt_path(:,2),'LineWidth',4);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % z 축의 데이터 도출
    
    zd = [];
    z_data = [];
    
    for i = 1 : length(xd)
        
        pos_x = 10000;
        pos_y = 10000;
        
        x_data = 0;
        y_data = 0;
        
        % 가장 가까운 위치로 지정
        for j = 1 : length(x_s)
           
            x_length = abs( x_s(j) - xd(i) );
            y_length = abs( y_s(j) - yd(i) );
            
            if( pos_x > x_length )
                pos_x = x_length;
                
                x_data = x_s(j);
            else
                pos_x = pos_x;
                x_data = x_data;
            end
            
            if( pos_y > y_length )
                pos_y = y_length;
                y_data = y_s(j);
            else
                pos_y = pos_y;
                y_data = y_data;
            end
            
        end
        
        % 해당 위치의 x좌표의 값과 y좌표의 값을 찾아 저장
        temp_pos_x = find( x_data == x_2d_s );
        temp_pos_y = find( y_data == y_2d_s );
        temp_pos_z = 0;
        
        for k = 1 : length(temp_pos_x)
            temp = find( temp_pos_x(k) == temp_pos_y );
            
            if( temp )
                temp_pos_z = temp;
                
                z_data(i) = z_2d_s(temp_pos_z);
                
                % 비행고도 제한 계산
                if( z_data(i) < flight_height )
                    z_data(i) = flight_height + height;
                end
                
            else
                temp_pos_z = temp_pos_z;
                
            end
            
        end
        
    end
    
    % 선회 직선 경로 데이터
    ttemp_xy_path_all{mainloof} = xy_opt_path;

    rootCount = 0;

    % 전체 경로 길이 구간별 계산
    path1 = 0;

    for i = 1 : length(xy_opt_path)-1

        path_part = sqrt((xy_opt_path(i,1) - xy_opt_path(i+1,1))^2 + (xy_opt_path(i,2) - xy_opt_path(i+1,2))^2 + ( (z_data(i+1)-z_data(i)) /1000)^2);
        path1 = path_part + path1;
        part_length(i) = path_part;

    end

    path_length(mainloof) = path1;
    part_length_all{mainloof} = part_length;

%     path_length_all = path_length(mainloof) + path_length_all;
    path_length_all = path1 + path_length_all;
    
        
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    
end

x = [31	32	33	34	35	36	37	38	39	40];
y = [80	79	79	78	78	77	77	76	76	75];
z = [700 850 900 950 980 990 900 800 800 700];

figure(2)
% making 3D MAP
meshc(x_2d_s, y_2d_s, z_2d_s);
hold on;

% using plot3 function   ex. plot3(x1, y1, z1, ...)
plot3(x, y, z + 5, 'red', 'LineWidth',4);

%%

if(path_length_all > lift_length)
    x_all = [x_all x_equation_1];
    y_all = [y_all y_equation_1];
    z_all = [z_all lift_height_1];
    
    disp('전체 상승 경로 길이');
    lift_length
    
%     length_all = length_all + lift_length

else
    x_all = [x_all xy_opt_path(:,1)'];
    y_all = [y_all xy_opt_path(:,2)'];
    
    z_temp = [];
    for i = 1 : length(xy_opt_path(:,1))
        z_temp(i) = flight_height;
        
    end
    
    z_all = [z_all z_temp];
    
    disp('전체 선회 경로 길이');
    path_length_all
    
%     length_all = length_all + path_length_all
    
end

% figure(2)
% % making 3D MAP
% meshc(x_2d_s, y_2d_s, z_2d_s);
% hold on;
% 
% % using plot3 function   ex. plot3(x1, y1, z1, ...)
% plot3(x_all, y_all, z_all + 10, 'black', 'LineWidth',4);

%% 8 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x_equation_1 = x_equation(39:41);
y_equation_1 = y_equation(39:41);
lift_height_1 = lift_height(39:41);

% 상승 및 하강 경로 총 길이
lift_length = 0;
%z_lift_height
lift_length_part = [];
%lift_heigth

for i = 1 : length(y_equation_1)-1
    lift_length_part(i) = sqrt( (x_equation_1(i+1)-x_equation_1(i))^2 + (y_equation_1(i+1)-y_equation_1(i))^2 + ( (lift_height_1(i+1)-lift_height_1(i))/1000 )^2 );

    lift_length = lift_length + sqrt( (x_equation_1(i+1)-x_equation_1(i))^2 + (y_equation_1(i+1)-y_equation_1(i))^2 + ( (lift_height_1(i+1)-lift_height_1(i))/1000 )^2 );

end

xpoint = [x_equation_1(1) x_equation_1(end)];
ypoint = [y_equation_1(1) y_equation_1(end)];

order = [1 2];

for mainloof = 1 : length(order)-1

    xy_start = [xpoint(order(mainloof)) ypoint(order(mainloof))];
    xy_dest = [xpoint(order(mainloof+1)) ypoint(order(mainloof+1))];
    
    %상승 및 하강
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % 추가 
    
    x_obs_pos = []; 
    y_obs_pos = [];
    
    x_obs_location = [];
    y_obs_location = [];
    
    obs_pos = [];
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % 장애물 위치 파악
    x_obs = [];
    y_obs = [];
    
    count_obs = 0;
    
    for i = 1 : length(x_equation)
        
        x_obs_pos = find(x_equation(i) == xn_new);
        y_obs_pos = find(y_equation(i) == yn_new);
        
    end
    
    for i = 1 : length(x_obs_pos)
        obs_pos = find(x_obs_pos(i) == y_obs_pos);
        %obs_pos
        
        if(obs_pos)
            x_obs_location(i) = xn_new(x_obs_pos(obs_pos));
            y_obs_location(i) = yn_new(y_obs_pos(obs_pos));
            
        end
         
    end
    
    % 여기 까지
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
    
    figure(1);
    plot(xy_start(1),xy_start(2),'S',xy_dest(1),xy_dest(2),'D','MarkerSize',10, 'MarkerFaceColor','red');
    plot(xy_opt_path(:,1),xy_opt_path(:,2),'LineWidth',4);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % z 축의 데이터 도출
    
    zd = [];
    z_data = [];
    
    for i = 1 : length(xd)
        
        pos_x = 10000;
        pos_y = 10000;
        
        x_data = 0;
        y_data = 0;
        
        % 가장 가까운 위치로 지정
        for j = 1 : length(x_s)
           
            x_length = abs( x_s(j) - xd(i) );
            y_length = abs( y_s(j) - yd(i) );
            
            if( pos_x > x_length )
                pos_x = x_length;
                
                x_data = x_s(j);
            else
                pos_x = pos_x;
                x_data = x_data;
            end
            
            if( pos_y > y_length )
                pos_y = y_length;
                y_data = y_s(j);
            else
                pos_y = pos_y;
                y_data = y_data;
            end
            
        end
        
        % 해당 위치의 x좌표의 값과 y좌표의 값을 찾아 저장
        temp_pos_x = find( x_data == x_2d_s );
        temp_pos_y = find( y_data == y_2d_s );
        temp_pos_z = 0;
        
        for k = 1 : length(temp_pos_x)
            temp = find( temp_pos_x(k) == temp_pos_y );
            
            if( temp )
                temp_pos_z = temp;
                
                z_data(i) = z_2d_s(temp_pos_z);
                
                % 비행고도 제한 계산
                if( z_data(i) < flight_height )
                    z_data(i) = flight_height + height;
                end
                
            else
                temp_pos_z = temp_pos_z;
                
            end
            
        end
        
    end
    
    % 선회 직선 경로 데이터
    ttemp_xy_path_all{mainloof} = xy_opt_path;

    rootCount = 0;

    % 전체 경로 길이 구간별 계산
    path1 = 0;

    for i = 1 : length(xy_opt_path)-1

        path_part = sqrt((xy_opt_path(i,1) - xy_opt_path(i+1,1))^2 + (xy_opt_path(i,2) - xy_opt_path(i+1,2))^2 + ( (z_data(i+1)-z_data(i)) /1000)^2);
        path1 = path_part + path1;
        part_length(i) = path_part;

    end

    path_length(mainloof) = path1;
    part_length_all{mainloof} = part_length;

%     path_length_all = path_length(mainloof) + path_length_all;
    path_length_all = path1 + path_length_all;
    
        
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    
end

%%

if(path_length_all > lift_length)
    x_all = [x_all x_equation_1];
    y_all = [y_all y_equation_1];
    z_all = [z_all lift_height_1];
    
    disp('전체 상승 경로 길이');
    lift_length
    
    length_all = length_all + lift_length

else
    x_all = [x_all xy_opt_path(:,1)'];
    y_all = [y_all xy_opt_path(:,2)'];
    
    z_temp = [];
    for i = 1 : length(xy_opt_path(:,1))
        z_temp(i) = flight_height;
        
    end
    
    z_all = [z_all z_temp];
    
    disp('전체 선회 경로 길이');
    path_length_all
    
    length_all = length_all + path_length_all
    
end

%% 9 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 최대 높이를 넘음

x_equation_1 = x_equation(41:65);
y_equation_1 = y_equation(41:65);
lift_height_1 = lift_height(41:65);

lift_length = 100000000000000000000;

% % 상승 및 하강 경로 총 길이
% lift_length = 0;
% %z_lift_height
% lift_length_part = [];
% %lift_heigth
% 
% for i = 1 : length(y_equation_1)-1
%     lift_length_part(i) = sqrt( (x_equation_1(i+1)-x_equation_1(i))^2 + (y_equation_1(i+1)-y_equation_1(i))^2 + ( (lift_height_1(i+1)-lift_height_1(i))/1000 )^2 );
% 
%     lift_length = lift_length + sqrt( (x_equation_1(i+1)-x_equation_1(i))^2 + (y_equation_1(i+1)-y_equation_1(i))^2 + ( (lift_height_1(i+1)-lift_height_1(i))/1000 )^2 );
% 
% end

xpoint = [x_equation_1(1) x_equation_1(end)];
ypoint = [y_equation_1(1) y_equation_1(end)];

order = [1 2];

for mainloof = 1 : length(order)-1

    xy_start = [xpoint(order(mainloof)) ypoint(order(mainloof))];
    xy_dest = [xpoint(order(mainloof+1)) ypoint(order(mainloof+1))];
    
    %상승 및 하강
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % 추가 
    
    x_obs_pos = []; 
    y_obs_pos = [];
    
    x_obs_location = [];
    y_obs_location = [];
    
    obs_pos = [];
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % 장애물 위치 파악
    x_obs = [];
    y_obs = [];
    
    count_obs = 0;
    
    for i = 1 : length(x_equation)
        
        x_obs_pos = find(x_equation(i) == xn_new);
        y_obs_pos = find(y_equation(i) == yn_new);
        
    end
    
    for i = 1 : length(x_obs_pos)
        obs_pos = find(x_obs_pos(i) == y_obs_pos);
        %obs_pos
        
        if(obs_pos)
            x_obs_location(i) = xn_new(x_obs_pos(obs_pos));
            y_obs_location(i) = yn_new(y_obs_pos(obs_pos));
            
        end
         
    end
    
    % 여기 까지
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
    
    figure(1);
    plot(xy_start(1),xy_start(2),'S',xy_dest(1),xy_dest(2),'D','MarkerSize',10, 'MarkerFaceColor','red');
    plot(xy_opt_path(:,1),xy_opt_path(:,2),'LineWidth',4);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % z 축의 데이터 도출
    
    zd = [];
    z_data = [];
    
    for i = 1 : length(xd)
        
        pos_x = 10000;
        pos_y = 10000;
        
        x_data = 0;
        y_data = 0;
        
        % 가장 가까운 위치로 지정
        for j = 1 : length(x_s)
           
            x_length = abs( x_s(j) - xd(i) );
            y_length = abs( y_s(j) - yd(i) );
            
            if( pos_x > x_length )
                pos_x = x_length;
                
                x_data = x_s(j);
            else
                pos_x = pos_x;
                x_data = x_data;
            end
            
            if( pos_y > y_length )
                pos_y = y_length;
                y_data = y_s(j);
            else
                pos_y = pos_y;
                y_data = y_data;
            end
            
        end
        
        % 해당 위치의 x좌표의 값과 y좌표의 값을 찾아 저장
        temp_pos_x = find( x_data == x_2d_s );
        temp_pos_y = find( y_data == y_2d_s );
        temp_pos_z = 0;
        
        for k = 1 : length(temp_pos_x)
            temp = find( temp_pos_x(k) == temp_pos_y );
            
            if( temp )
                temp_pos_z = temp;
                
                z_data(i) = z_2d_s(temp_pos_z);
                
                % 비행고도 제한 계산
                if( z_data(i) < flight_height )
                    z_data(i) = flight_height + height;
                end
                
            else
                temp_pos_z = temp_pos_z;
                
            end
            
        end
        
    end
    
    % 선회 직선 경로 데이터
    ttemp_xy_path_all{mainloof} = xy_opt_path;

    rootCount = 0;

    % 전체 경로 길이 구간별 계산
    path1 = 0;

    for i = 1 : length(xy_opt_path)-1

        path_part = sqrt((xy_opt_path(i,1) - xy_opt_path(i+1,1))^2 + (xy_opt_path(i,2) - xy_opt_path(i+1,2))^2 + ( (z_data(i+1)-z_data(i)) /1000)^2);
        path1 = path_part + path1;
        part_length(i) = path_part;

    end

    path_length(mainloof) = path1;
    part_length_all{mainloof} = part_length;

%     path_length_all = path_length(mainloof) + path_length_all;
    path_length_all = path1 + path_length_all;
    
        
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    
end

%%

if(path_length_all > lift_length)
    x_all = [x_all x_equation_1];
    y_all = [y_all y_equation_1];
    z_all = [z_all lift_height_1];
    
    disp('전체 상승 경로 길이');
    lift_length
    
    length_all = length_all + lift_length

else
    x_all = [x_all xy_opt_path(:,1)'];
    y_all = [y_all xy_opt_path(:,2)'];
    
    z_temp = [];
    for i = 1 : length(xy_opt_path(:,1))
        z_temp(i) = flight_height;
        
    end
    
    z_all = [z_all z_temp];
    
    disp('전체 선회 경로 길이');
    path_length_all
    
    length_all = length_all + path_length_all
    
end

%% 10 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x_equation_1 = x_equation(65:end);
y_equation_1 = y_equation(65:end);
lift_height_1 = lift_height(65:end);

lift_length = 100000000000;

% % 상승 및 하강 경로 총 길이
% lift_length = 0;
% %z_lift_height
% lift_length_part = [];
% %lift_heigth
% 
% for i = 1 : length(y_equation_1)-1
%     lift_length_part(i) = sqrt( (x_equation_1(i+1)-x_equation_1(i))^2 + (y_equation_1(i+1)-y_equation_1(i))^2 + ( (lift_height_1(i+1)-lift_height_1(i))/1000 )^2 );
% 
%     lift_length = lift_length + sqrt( (x_equation_1(i+1)-x_equation_1(i))^2 + (y_equation_1(i+1)-y_equation_1(i))^2 + ( (lift_height_1(i+1)-lift_height_1(i))/1000 )^2 );
% 
% end

xpoint = [x_equation_1(1) x_equation_1(end)];
ypoint = [y_equation_1(1) y_equation_1(end)];

order = [1 2];

for mainloof = 1 : length(order)-1

    xy_start = [xpoint(order(mainloof)) ypoint(order(mainloof))];
    xy_dest = [xpoint(order(mainloof+1)) ypoint(order(mainloof+1))];
    
    %상승 및 하강
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % 추가 
    
    x_obs_pos = []; 
    y_obs_pos = [];
    
    x_obs_location = [];
    y_obs_location = [];
    
    obs_pos = [];
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % 장애물 위치 파악
    x_obs = [];
    y_obs = [];
    
    count_obs = 0;
    
    for i = 1 : length(x_equation)
        
        x_obs_pos = find(x_equation(i) == xn_new);
        y_obs_pos = find(y_equation(i) == yn_new);
        
    end
    
    for i = 1 : length(x_obs_pos)
        obs_pos = find(x_obs_pos(i) == y_obs_pos);
        %obs_pos
        
        if(obs_pos)
            x_obs_location(i) = xn_new(x_obs_pos(obs_pos));
            y_obs_location(i) = yn_new(y_obs_pos(obs_pos));
            
        end
         
    end
    
    % 여기 까지
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
    
    figure(1);
    plot(xy_start(1),xy_start(2),'S',xy_dest(1),xy_dest(2),'D','MarkerSize',10, 'MarkerFaceColor','red');
    plot(xy_opt_path(:,1),xy_opt_path(:,2),'LineWidth',4);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % z 축의 데이터 도출
    
    zd = [];
    z_data = [];
    
    for i = 1 : length(xd)
        
        pos_x = 10000;
        pos_y = 10000;
        
        x_data = 0;
        y_data = 0;
        
        % 가장 가까운 위치로 지정
        for j = 1 : length(x_s)
           
            x_length = abs( x_s(j) - xd(i) );
            y_length = abs( y_s(j) - yd(i) );
            
            if( pos_x > x_length )
                pos_x = x_length;
                
                x_data = x_s(j);
            else
                pos_x = pos_x;
                x_data = x_data;
            end
            
            if( pos_y > y_length )
                pos_y = y_length;
                y_data = y_s(j);
            else
                pos_y = pos_y;
                y_data = y_data;
            end
            
        end
        
        % 해당 위치의 x좌표의 값과 y좌표의 값을 찾아 저장
        temp_pos_x = find( x_data == x_2d_s );
        temp_pos_y = find( y_data == y_2d_s );
        temp_pos_z = 0;
        
        for k = 1 : length(temp_pos_x)
            temp = find( temp_pos_x(k) == temp_pos_y );
            
            if( temp )
                temp_pos_z = temp;
                
                z_data(i) = z_2d_s(temp_pos_z);
                
                % 비행고도 제한 계산
                if( z_data(i) < flight_height )
                    z_data(i) = flight_height + height;
                end
                
            else
                temp_pos_z = temp_pos_z;
                
            end
            
        end
        
    end
    
    % 선회 직선 경로 데이터
    ttemp_xy_path_all{mainloof} = xy_opt_path;

    rootCount = 0;

    % 전체 경로 길이 구간별 계산
    path1 = 0;

    for i = 1 : length(xy_opt_path)-1

        path_part = sqrt((xy_opt_path(i,1) - xy_opt_path(i+1,1))^2 + (xy_opt_path(i,2) - xy_opt_path(i+1,2))^2 + ( (z_data(i+1)-z_data(i)) /1000)^2);
        path1 = path_part + path1;
        part_length(i) = path_part;

    end

    path_length(mainloof) = path1;
    part_length_all{mainloof} = part_length;

%     path_length_all = path_length(mainloof) + path_length_all;
    path_length_all = path1 + path_length_all;
    
        
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    
end

%%

if(path_length_all > lift_length)
    x_all = [x_all x_equation_1];
    y_all = [y_all y_equation_1];
    z_all = [z_all lift_height_1];
    
    disp('전체 상승 경로 길이');
    lift_length
    
    length_all = length_all + lift_length

else
    x_all = [x_all xy_opt_path(:,1)'];
    y_all = [y_all xy_opt_path(:,2)'];
    
    z_temp = [];
    for i = 1 : length(xy_opt_path(:,1))
        z_temp(i) = flight_height;
        
    end
    
    z_all = [z_all z_temp];
    
    disp('전체 선회 경로 길이');
    path_length_all
    
    length_all = length_all + path_length_all
    
end

figure(2)
% making 3D MAP
meshc(x_2d_s, y_2d_s, z_2d_s);
hold on;

% using plot3 function   ex. plot3(x1, y1, z1, ...)
plot3(x_all, y_all, z_all + 10, 'red', 'LineWidth',4);




























