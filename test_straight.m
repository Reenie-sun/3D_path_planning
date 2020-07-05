clear;
clc;

N = 20;
x = linspace(1, N, N);
y = linspace(1, N, N);

x_2d = repmat(x, N, 1);
y_2d = repmat(y', 1, N);

x_1d = zeros(1, N*N);
y_1d = zeros(1, N*N);

z_1d = zeros(1, N*N);
z_2d = zeros(N, N);

Z = zeros(N, N);
% 1 ~ 15사이의 난수 N by N 생성
Z = randi(15, N, N);

cnt = 1;
for i = 1: N %1자로 만드는부분
    for j = 1: N
        x_1d(cnt) = x_2d(i,j);
        y_1d(cnt) = y_2d(i,j);
        z_1d(cnt) = Z(i,j);
        z_2d(i,j) = Z(i,j);
        cnt = cnt+1;
    end
end

figure(1)
% making 3D MAP
meshc(x_2d, y_2d, z_2d);
hold on;

xpoint = [2; 18];
ypoint = [3; 15];

% 운항고도
flight_height = 10;

% 최대 운항고도
max_flight_height = 13;


%% 기존 프로그램 추가
% i_site = find(z_1d_s > flight_height);  %높이 값주는부분 실제로 사용될 크기 판단
% xn_new = zeros(1,length(i_site));  
% yn_new = zeros(1,length(i_site));  
% z_new = zeros(1,length(i_site));
% 
% for sitetmp = 1:length(i_site) % 높이에 의해서 걸러진 xy위치 저장
%    xn_new(sitetmp) = x_1d_s(i_site(sitetmp));
%    yn_new(sitetmp) = y_1d_s(i_site(sitetmp)); 
%    z_new(sitetmp) = z_1d_s(i_site(sitetmp)); 
% end
% 
% [vx, vy] = voronoi(xn_new,yn_new);
% 
% % 1열의 데이터 획득
% vx1 = vx(1,:); 
% vy1 = vy(1,:);
% 
% % 2열의 데이터 획득
% vx2 = vx(2,:);
% vy2 = vy(2,:);
% 
% c_r = 1;
% vx_new = vx;
% vy_new = vy;
% 
% for i = 1 : length(xn_new)
%     % 1열의 데이터 획득
%     vx1 = vx_new(1,:); 
%     vy1 = vy_new(1,:);
% 
%     % 2열의 데이터 획득
%     vx2 = vx_new(2,:);
%     vy2 = vy_new(2,:);
% 
%     r_sq = (vx1-xn_new(i)).^2+(vy1-yn_new(i)).^2;
%     idx1 = find((r_sq < c_r^2));
%     
%     r_sq = (vx2-xn_new(i)).^2+(vy2-yn_new(i)).^2;
%     idx2 = find((r_sq < c_r^2));
%     
%     idx = union(idx1,idx2);
%     
%    
%     vx_new(:,idx) = [];
%     vy_new(:,idx) = [];
%     
% end
% 
% %4단계 샘사이트 경로 표기
% th = 0:0.01:2*pi; %원을 그리기위한 각도 배열
% %c_cx = 15; c_cy = 17; c_r =5;
% c_cx = 0; c_cy = 0; c_r = 0;
% xc = c_r*cos(th)+c_cx; 
% yc = c_r*sin(th)+c_cy;
% 
% vx1 = vx_new(1,:); 
% vy1 = vy_new(1,:);
% r_sq = (vx1-c_cx).^2+(vy1-c_cy).^2;
% idx1 = find((r_sq < c_r^2));
% 
% vx2 = vx_new(2,:);
% vy2 = vy_new(2,:);
% 
% r_sq = (vx2-c_cx).^2+(vy2-c_cy).^2;
% idx2 = find((r_sq < c_r^2));
% 
% idx = union(idx1,idx2);
% 
% vx_new(:,idx) = [];
% vy_new(:,idx) = [];

%% 직선상의 상승을 이용한 경로 선정(직선상의 x, y, z 값 추출)

% 직선의 방정식 : y = ax + b
res = polyfit(xpoint, ypoint, 1);
x_equation = xpoint(1) : (x(2)-x(1)) : xpoint(2);
y_equation = res(1)*x_equation + res(2);

z_lift_height = [];

y_index = [];

% 직선 경로에 있는 점들중 일치하는 y 지점 찾기
for i = 1 : length(y_equation)

    flag = 0;
    
    for j = 1 : N
        num = find( y_equation(i) == y(j) );

        if(num)
            y_equation(i) = y(j);
            flag = 1;
            y_index(i) = j;
            break;

        else
            flag = 0;

        end
        
    end
    
    if(flag == 0)
        y_equation(i) = y(knnsearch(y', y_equation(i)));
        y_index(i) = knnsearch(y', y_equation(i));
    end
    
    %y_index(i) = knnsearch(y', y_equation(i));

end

% x_equation
% y_equation

lift_height = [];

obs_index = [];

% 장애물의 개수
count_obs = 0;

% 직선 경로에 있는 점들중 일치하는 z 지점 찾기
for k = 1 : length(y_equation)
    for i = 1 : N
        
        % x와 동일한 지점 2d에서 찾기
        if(find( x_equation(k) == x(i) ))
            z_lift_height(k) = z_2d( (y_index(k)-1)*N + i );
            
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

% z_lift_height

plot3(x_equation, y_equation, lift_height + 2, 'black', 'LineWidth',4);

%% 상승각 구하기

obs_index

% 장애물의 위치 정보 저장
x_obs = [];
y_obs = [];

th = [];

x_equation
y_equation
lift_height

% for k = 1 : length(y_equation)
%         
%     if((lift_height(k)-3) == flight_height)
%         % 장애물 위치 파악
% %         count_obs = count_obs + 1;
%         x_obs(count_obs) = x_equation(k);
%         y_obs(count_obs) = y_equation(k);
%         
%         z_obs(count_obs) = lift_height(k);
% 
%     end
% 
% end 

% k == obs_index(i)
for i = 1 : count_obs
    
    k = obs_index(i);

    if( lift_height(k) < lift_height(k+1) )
        disp('up');
        a = ( (y_equation(k+1) - y_equation(k)) / (x_equation(k+1) - x_equation(k)) );
        th_temp = atan(a);
        % radian -> degree
        th(i) = th_temp * 180 / pi;
        
    elseif( lift_height(k) > lift_height(k+1) )
        disp('down');
        
    else
        disp('nothing');
        
    end

end

% x_obs
% y_obs
% z_obs

count_obs

th
%% 상승각 구하기

% % th = findLiftAngle(x_obs, y_obs, z_obs, count_obs)
% 
% 
% 
% % count_obs = 3;
% % 
% % x_obs = [6 8 11 13 14 15 16];
% % y_obs = [6 7 10 11 12 13 13];
% % z_obs = [10 11 14 12 9 11 14];
% 
% l = 1;
% 
% % count_obs = 0;
% 
% % 장애물 개수만큼 검사 진행
% for i = 1 : count_obs
% %     for i = 1 : length(z_obs)
%         
%         for k = 1 : length(y_equation)-1
%         
% %             if((lift_height(k)-3) == flight_height)
%             
%                 if(lift_height(k) == z_obs(i))
%                     
%                     if(lift_height(k+1) > lift_height(k))
%                         a = ( (y_equation(k+1) - y_equation(k)) / (x_equation(k+1) - x_equation(k)) );
%                         th_temp = atan(a);
%                         % radian -> degree
%                         th(l) = th_temp * 180 / pi;
%                         l = l + 1;
% 
%                         display('hello');
% 
%                         break;
%                     end
% 
%                 end
%                 
% %             end
% 
%         end
% 
% %     end
% end
% 
% count_obs
% th

%% 상승 및 하강 경로 선정 알고리즘 수행

% 선회 경로 총 길이
path_length_all = 0;

part_length = [];

% 최대 상승 높이(최대 운용 고도)
max_flight_height = 1000;



% 최대 운항 높이를 넘지 않도록 함
if(max_flight_height < flight_height)
    % Do voronoi

    % 2포인트에 대하여 수정
    if(length(xpoint) > 3)
        order = tsp(xpoint, ypoint);

    else
        order = [1 2];

    end




else
    % Do Lift()
    % z축 간격
    z_alpha = 5;

    plot3(x_equation, y_equation, lift_height +z_alpha, 'black', 'LineWidth',4);

    
    % 상승 및 하강 경로 총 길이
    lift_length = 0;
    %z_lift_height
    lift_length_part = [];
    %lift_heigth

    for i = 1 : length(y_equation)-1
        lift_length_part(i) = sqrt( (x_equation(i+1)-x_equation(i))^2 + (y_equation(i+1)-y_equation(i))^2 + ( (lift_height(i+1)-lift_height(i))/1000 )^2 );

        lift_length = lift_length + sqrt( (x_equation(i+1)-x_equation(i))^2 + (y_equation(i+1)-y_equation(i))^2 + ( (lift_height(i+1)-lift_height(i))/1000 )^2 );

    end

    disp('전체 상승 경로 길이');
    lift_length

end
























