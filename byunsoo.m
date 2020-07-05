%석사 학위 시뮬레이터입니다.

clear;


[Z, ref] = dted('n37.dt0');

N_s = 100;
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
% figure(1);
% set(gcf,'numbertitle','off','name', '등고선지도');
% contour(x_2d_s,y_2d_s,z_2d_s,'ShowText','on');
% xlabel('km');
% ylabel('km');


%2단계 위협표기

i_site = find(z_1d_s > 700);  %높이 값주는부분 실제로 사용될 크기 판단
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

% figure(2);
% set(gcf,'numbertitle','off','name', '위협표기');
% contour(x_2d_s,y_2d_s,z_2d_s,'ShowText','on');
% xlabel('km');
% ylabel('km');
% hold on;
% %plot(vx_new,vy_new,'b.-')
% plot(xn_new,yn_new,'r.');
% axis([0 N_s 0 N_s]);


%3단계 운항가능 경로 표기

% figure(3);
% set(gcf,'numbertitle','off','name', '경로표기');
% contour(x_2d_s,y_2d_s,z_2d_s,'ShowText','on');
% xlabel('km');
% ylabel('km');
% hold on;
% plot(vx_new,vy_new,'b.-')
% plot(xn_new,yn_new,'r.');
% axis([0 N_s 0 N_s]);

%4단계 샘사이트 경로 표기
th = 0:0.01:2*pi; %원을 그리기위한 각도 배열
c_cx = 15; c_cy = 17; c_r =5;
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

% figure(4);
% set(gcf,'numbertitle','off','name', '샘사이트반영');
% contour(x_2d_s,y_2d_s,z_2d_s,'ShowText','on');
% xlabel('km');
% ylabel('km');
% hold on;
% plot(vx_new,vy_new,'b.-')
% plot(xn_new,yn_new,'r.');
% h = fill(xc,yc,'red');
% set(h,'facealpha',.5)
% axis([0 N_s 0 N_s]);


%5단계 최종경로

xpoint = [10;49;22;46;14;30];
ypoint = [10;42;53;13;29;17];

order = tsp(xpoint,ypoint);

% figure(5);
set(gcf,'numbertitle','off','name', '최종경로');
hold on;

lgd = legend('출발점','도착점');
lgd.AutoUpdate = 'off';
contour(x_2d_s,y_2d_s,z_2d_s,'ShowText','on');
xlabel('km');
ylabel('km');

plot(vx_new,vy_new,'b.-')
plot(xn_new,yn_new,'r.');
h = fill(xc,yc,'red');
set(h,'facealpha',.5)
axis([0 N_s 0 N_s]);



for mainloof = 1 : length(order)-1

    xy_start = [xpoint(order(mainloof)) ypoint(order(mainloof))];
    xy_dest = [xpoint(order(mainloof+1)) ypoint(order(mainloof+1))];
    
    xy_opt_path = F_XY_OPT_PATH(xy_start, xy_dest, vx_new, vy_new, xn_new, yn_new, xc, yc);

%     vx_all = vx_new(:);
%      vy_all = vy_new(:);
% 
%     dr = kron(ones(length(vx_all),1),xy_start)-[vx_all vy_all];
%     [min_val,min_id] = min(sum(dr.^2,2));
% 
%     vx_new = [vx_new [xy_start(1); vx_all(min_id)]];
%     vy_new = [vy_new [xy_start(2); vy_all(min_id)]];
% 
%     dr = kron(ones(length(vx_all),1),xy_dest)-[vx_all vy_all];
%     [min_val,min_id] = min(sum(dr.^2,2));
% 
%     vx_new = [vx_new [xy_dest(1); vx_all(min_id)]];
%     vy_new = [vy_new [xy_dest(2); vy_all(min_id)]];
% 
%     xy_all = unique([vx_new(:) vy_new(:)],'rows');
%     dv = [vx_new(1,:); vy_new(1,:)] - [vx_new(2,:); vy_new(2,:)];
%     edge_dist = sqrt(sum(dv.^2));
% 
%     G = sparse(size(xy_all,1),size(xy_all,1));
% 
%     for kdx = 1:length(edge_dist)
%         xy_s = [vx_new(1,kdx) vy_new(1,kdx)];
%         idx = find(sum((xy_all-kron(ones(size(xy_all,1),1),xy_s)).^2,2)==0);
%         xy_d = [vx_new(2,kdx) vy_new(2,kdx)];
%         jdx = find(sum((xy_all-kron(ones(size(xy_all,1),1),xy_d)).^2,2)==0);
%         G(idx,jdx) = edge_dist(kdx);
%         G(jdx,idx) = edge_dist(kdx);
%     end
% 
%     st_idx = find(sum((xy_all-kron(ones(size(xy_all,1),1),xy_start)).^2,2)==0);
%     dest_idx = find(sum((xy_all-kron(ones(size(xy_all,1),1),xy_dest)).^2,2)==0);
% 
%     [dist,paths,pred] = graphshortestpath(G,st_idx,dest_idx);
%     xy_opt_path = xy_all(paths,:);
%     
%     %1열의 데이터 획득
%     d_r = 1.4;
%     d_r2 = 0.9;
%     xd = xy_opt_path(:,1); 
%     yd = xy_opt_path(:,2);
%     trues = zeros(1,length(xy_opt_path));
%     for i = 1 : length(xy_opt_path)
%         r_sq = (xd(i)-xn_new).^2+(yd(i)-yn_new).^2;
%         idx1 = find((r_sq < d_r^2));
%         r_sq = (xd(i)-xc).^2+(yd(i)-yc).^2;
%         idx2 = find((r_sq < d_r2^2));
% 
%         if (isempty(idx1)==1) && (isempty(idx2)==1)
%             trues(i) = 1;
%         end
%     end
    plot(xy_start(1),xy_start(2),'S',xy_dest(1),xy_dest(2),'D','MarkerSize',10, 'MarkerFaceColor','red')
    plot(xy_opt_path(:,1),xy_opt_path(:,2), 'k', 'LineWidth',3);
         
%     for i = 1 : length(xy_opt_path)
%         if trues(i) == 0
%             xx = [xy_opt_path(i,1) xy_opt_path(i+1,1)];
%             yy = [xy_opt_path(i,2) xy_opt_path(i+1,2)];
%             plot(xx,yy, 'r-','LineWidth',3);
%         end
%     end
    %나중에 해결
end



    %%%%%%%%%%%%%%% function
    
drone_v = 70;
drone_t = 2;

temp_xpoint = xpoint;
temp_ypoint = ypoint;


[drone_x, drone_count, path_value] = dronePathSolution_cell(xpoint, ypoint, drone_v, drone_t, vx_new, vy_new, xn_new, yn_new, xc, yc);

drone_x
drone_count
path_value


% %%%%%%%%%%%%%%%%%% function 내부
% 
% drone_s = drone_v * drone_t;
% 
% drone_count = 0;
% path_value = 0; 
% 
% b_temp_x = [];
% b_temp_y = [];
% 
% drone_x = 0;
% 
% 
% while (length(temp_xpoint) >= 2)
%     
%     path_all = 0;
%     
%     if(length(temp_xpoint) < 3)
%                 
%         %%%%%%%%%%%%%%%%%%%%%%%     경로 계산
%         xy_start = [temp_xpoint(temp_order(1)) temp_ypoint(temp_order(1))];
%         xy_dest = [temp_xpoint(temp_order(2)) temp_ypoint(temp_order(2))];
% 
%         xy_opt_path = F_XY_OPT_PATH(xy_start, xy_dest, vx_new, vy_new, xn_new, yn_new, xc, yc);
%         
%         % 전체 경로 길이 구간별 계산
%         path_all = 0;
%         
%         for i = 1 : length(xy_opt_path)-1
%             path_all = sqrt((xy_opt_path(i,1) - xy_opt_path(i+1,1))^2 + (xy_opt_path(i,2) - xy_opt_path(i+1,2))^2) + path_all;
% 
%         end
%         
%         path_all = 2 * path_all;
%         
%         %%%%%%%%%%%%%%%%%%%%%%
% 
%         if(path_all < drone_s)
%         
% %             disp('한줄 그렸다');
%             
%             plot(xy_opt_path(:,1), xy_opt_path(:,2), 'LineWidth', 3);
% 
%             drone_count = drone_count + 1;
%             
%             path_value(drone_count) = path_all;
%             
%         else
%             disp('경로의 길이가 부족합니다.');
%             path_all
%             
%             drone_x = drone_x + 1;
%             
%         end
%                 
%         temp_xpoint = b_temp_x;
%         temp_ypoint = b_temp_y;
%         
%         b_temp_x = [];
%         b_temp_y = [];        
%         
%         
%     else
%     
%         temp_order = tsp(temp_xpoint,temp_ypoint);
%         
%         tpath = 0;
%         temp_xy = [];
%         xy_opt = [];
%                 
%         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%       경로 계산
%         for mainloof = 1: length(temp_xpoint)
%             
%             % 마지막 부분
%             if(mainloof == length(temp_xpoint))
%                 %%%%%%%%%%%%%
% 
%                 xy_start = [temp_xpoint(temp_order(end-1)) temp_ypoint(temp_order(end-1))];
%                 xy_dest = [temp_xpoint(temp_order(1)) temp_ypoint(temp_order(1))];
% 
%                 %%%%%%%%%%%%% 
%                 
%             else
%             
%                 %%%%%%%%%%%%%
% 
%                 xy_start = [temp_xpoint(temp_order(mainloof)) temp_ypoint(temp_order(mainloof))];
%                 xy_dest = [temp_xpoint(temp_order(mainloof+1)) temp_ypoint(temp_order(mainloof+1))];
% 
%                 %%%%%%%%%%%%%
%             
%             end
%             
%             xy_opt_path = F_XY_OPT_PATH(xy_start, xy_dest, vx_new, vy_new, xn_new, yn_new, xc, yc);
%             
%             % 전체 경로 길이 구간별 계산
%             tpath = 0;
% 
%             for i = 1 : length(xy_opt_path)-1
%                 tpath = sqrt((xy_opt_path(i,1) - xy_opt_path(i+1,1))^2 + (xy_opt_path(i,2) - xy_opt_path(i+1,2))^2) + tpath;
% 
%             end
%             
%             path_all = path_all + tpath;
%                 
%             path_aa(mainloof) = tpath;
% 
%             temp_xy{mainloof} = xy_opt_path;
%             
% 
%         end % for mainloof = 1: length(temp_order)-1 end
%         
%         %%%%%%%%%%%%%%%%%%%%%%%%%%%     경로 계산 끝   
% 
%         if(path_all < drone_s)
%             
% %             disp('그렸다');
% 
%             xy_opt = temp_xy{1};
%             
%             for jj = 2 : length(temp_xpoint)
%                 test = temp_xy{jj};
%                 xy_opt = [xy_opt ; test];
%                               
%             end
%             
%             plot(xy_opt(:,1), xy_opt(:,2), 'LineWidth', 3);
%             
% %             path_all
%                      
%             drone_count = drone_count + 1;
%             
%             path_value(drone_count) = path_all;
%             
%             temp_xpoint = b_temp_x;
%             temp_ypoint = b_temp_y;
%             
%             b_temp_x = [];
%             b_temp_y = [];
% 
%         else
% 
%             half_order = ceil(length(temp_order)/2);
% 
%             porder = temp_order(1:half_order);
%             border = temp_order(half_order+1:end);
%             
%             path1 = 0;
%             path2 = 0;
%             
%             xy_opt_path1 = temp_xy{half_order-1};
%             xy_opt_path2 = temp_xy{half_order};
%             
%             if (length(porder) > 2)
% 
%                 for i = 1 : length(xy_opt_path1)-1
%                     path1 = sqrt((xy_opt_path1(i,1) - xy_opt_path1(i+1,1))^2 + (xy_opt_path1(i,2) - xy_opt_path1(i+1,2))^2) + path1;
%                 end
% 
%                 for i = 1 : length(xy_opt_path2)-1
%                     path2 = sqrt((xy_opt_path2(i,1) - xy_opt_path2(i+1,1))^2 + (xy_opt_path2(i,2) - xy_opt_path2(i+1,2))^2) + path2;
%                 end
% 
%                 if path1 > path2
%                     border(end+1) = porder(end);
%                     porder(end) = [];
% 
%                 end
%                 
%             end
% 
%             border = sort(border);
%                         
%             % 비어있으면 1
%             if(isempty(b_temp_x) == 1)
%                 b_temp_x = temp_xpoint(border);
%                 b_temp_y = temp_ypoint(border);
%             else
%                 b_temp_x = [b_temp_x' temp_xpoint(border(2:end))'];
%                 b_temp_y = [b_temp_y' temp_ypoint(border(2:end))'];
%             end
%             
%             
%             temp_xpoint = temp_xpoint(porder);
%             temp_ypoint = temp_ypoint(porder);
% 
%         end
%         
%         
%     end
%         
% 
% end
% 
% drone_count
% path_value
% drone_x


% plot(xy_start(1),xy_start(2),'S',xy_dest(1),xy_dest(2),'D','MarkerSize',10, 'MarkerFaceColor','red')



