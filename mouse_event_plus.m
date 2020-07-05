%석사 학위 시뮬레이터입니다.

clear;


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

% 1단계 등고선지도 획득
%figure(1);
set(gcf,'numbertitle','off','name', '등고선지도');
contour(x_2d_s,y_2d_s,z_2d_s,'ShowText','on');
xlabel('km');
ylabel('km');


%% 2단계 위협표기

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

%figure(1);
set(gcf,'numbertitle','off','name', '위협표기');
contour(x_2d_s,y_2d_s,z_2d_s,'ShowText','on');
xlabel('km');
ylabel('km');
hold on;
%plot(vx_new,vy_new,'b.-')
plot(xn_new,yn_new,'r.');
axis([0 N_s 0 N_s]);


%% 3단계 운항가능 경로 표기

%figure(1);
set(gcf,'numbertitle','off','name', '경로표기');

contour(x_2d_s,y_2d_s,z_2d_s,'ShowText','on');
xlabel('km');
ylabel('km');
hold on;
plot(vx_new,vy_new,'b.-')
plot(xn_new,yn_new,'r.');
axis([0 N_s 0 N_s]);

%% 4단계 샘사이트 경로 표기
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

%figure(1);
set(gcf,'numbertitle','off','name', '샘사이트반영');
contour(x_2d_s,y_2d_s,z_2d_s,'ShowText','on');
xlabel('km');
ylabel('km');
hold on;
plot(vx_new,vy_new,'b.-')
plot(xn_new,yn_new,'r.');
h = fill(xc,yc,'red');
set(h,'facealpha',.5)
axis([0 N_s 0 N_s]);


%5단계 최종경로

%% 마우스로 좌표 넣는 부분
% xpoint = [10;49;22;46;14;30];
% ypoint = [10;42;53;13;29;17];
xpoint = [];
ypoint = [];

% 마우스로 지점 클릭 -> 우클릭시 좌표 입력 종료
button = 1;

%figure(1);
set(gcf,'numbertitle','off','name', '이동 지점 선택(샘사이트 반영)');
contour(x_2d_s,y_2d_s,z_2d_s,'ShowText','on');
xlabel('km');
ylabel('km');
hold on;
plot(vx_new,vy_new,'b.-')
plot(xn_new,yn_new,'r.');
h = fill(xc,yc,'red');
set(h,'facealpha',.5)
axis([0 N_s 0 N_s]);

posCount = 0;
i = 0;

while 1
    
    [xMouse, yMouse, button] = ginput(1);
    
    posCount = posCount + 1;
    
    if button == 3
        break
        
    else
        plot(xMouse, yMouse, 'rx', 'MarkerSize', 15);
        
        if i == 0
            c = '시작 지점';
            text(xMouse + 1, yMouse, c, 'FontSize', 15, 'Color', 'red', 'FontWeight', 'bold');
        else
            c = strcat(int2str(posCount-1), '번 지점');
            text(xMouse + 1, yMouse, c, 'FontSize', 15, 'Color', 'red', 'FontWeight', 'bold');
        end

        i = i + 1;
        
        xpoint(i) = xMouse;
        ypoint(i) = yMouse;
        
    end
    
end

xpoint = xpoint';
ypoint = ypoint';

%%
order = tsp(xpoint,ypoint);
 
%figure(1);
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
%order;

temp_xy_opt_path = {};

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
    
    plot(xy_start(1),xy_start(2),'S',xy_dest(1),xy_dest(2),'D','MarkerSize',10, 'MarkerFaceColor','red')
    
%     c = '시작 지점';
%     text(xpoint(1) - 5, ypoint(1) - 3, c, 'FontSize', 13, 'FontWeight', 'bold');
    
    plot(xy_opt_path(:,1),xy_opt_path(:,2),'LineWidth',4);
    
    %%%%%%%%%%%%%
        p = 2;

        while p < length(xy_opt_path)-2
            if length(xy_opt_path) > 5
                x1 = (xy_opt_path(p,1)+xy_opt_path(p+1,1))/2;       y1 = (xy_opt_path(p,2)+xy_opt_path(p+1,2))/2;   
                x2 = xy_opt_path(p+1,1);                            y2 = xy_opt_path(p+1,2) ; 
                x3 = (xy_opt_path(p+2,1)+xy_opt_path(p+1,1))/2;     y3  = (xy_opt_path(p+2,2)+xy_opt_path(p+1,2))/2; 

                a = [x1, y1, 1;           x2, y2, 1;           x3, y3, 1];
                d = [x1^2+y1^2, y1, 1;    x2^2+y2^2, y2, 1;    x3^2+y3^2, y3, 1];
                e = [x1^2+y1^2, x1, 1;    x2^2+y2^2, x2, 1;    x3^2+y3^2, x3, 1];
                f = [x1^2+y1^2, x1, y1;    x2^2+y2^2, x2, y2;    x3^2+y3^2, x3, y3];

                centerx = -(det(-d)/(2 * det(a)));
                centery = -(det(e)/(2 * det(a)));
                radius = sqrt( ((det(-d)^2 + det(e)^2) / (4*det(a)^2)) - (det(-f)/det(a)));

        %         seta1 = atan((y1-centery)/(x1-centerx))*180/pi
        %         seta2 = atan((y2-centery)/(x2-centerx))*180/pi
        %         seta3 = atan((y3-centery)/(x3-centerx))*180/pi
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

                useta = sort([seta1 seta2 seta3]);
                if (max(useta)-min(useta)) > 180
                     tempangle = 0 : 0.01 : 2*pi;
                     tempangle2 = min(useta)/180*pi:0.01:max(useta)/180*pi;
                     tempangle = fix(tempangle*10^2) / 10^2;
                     tempangle2 = fix(tempangle2*10^2) / 10^2;
                     thtemp = setdiff(tempangle,tempangle2);
                     snum = 0;
                     for ee = 1 : length(thtemp)-1
                         abs(thtemp(ee)-thtemp(ee+1));
                         if abs(thtemp(ee)-thtemp(ee+1))>0.03
                             snum = ee;
                             break;
                         end
                     end

                     findex = find(thtemp >= thtemp(ee+1));
                     seperate1 = thtemp(findex);
                     seperate2 = setdiff(thtemp,seperate1);
                     thtemp = [seperate1,seperate2];
        %             tempangle = useta(2)/180*pi:0.01:max(useta)/180*pi;
        %             tempangle2 = 0:0.01:min(useta)/180*pi;
        %             thtemp = [tempangle,tempangle2];
                else
                    thtemp = min(useta)/180*pi:0.01:max(useta)/180*pi; %원을 그리기위한 각도 배열
                end

                %thtemp = useta(3)/180*pi:0.01:useta(1)/180*pi; %원을 그리기위한 각도 배열
                c_cxt = centerx; c_cyt = centery; c_rt =radius;
                xct = c_rt*cos(thtemp)+c_cxt; 
                yct = c_rt*sin(thtemp)+c_cyt;
                plot(xct,yct,'red','LineWidth',4);
                p=p+1;

            end
        end
        
        
        %%%%%%%%%%%%%%
        
end
    
    
    
    
    
%     %% 궤적 이동 그림
%     
%     for i = 1 : length(xy_opt_path)-1
%             
%         xx = [xy_opt_path(i,1) xy_opt_path(i+1,1)];
%         yy = [xy_opt_path(i,2) xy_opt_path(i+1,2)];
% 
%         plot(xx, yy, 'k-','LineWidth',3);
% 
%         drawnow;
%             
%     end
%     
%     xy_opt_path;
%     length(xy_opt_path);
%     
%     % temp_xy_opt_path : cell data
%     temp_xy_opt_path{mainloof} = xy_opt_path;
%     
% end
% 
% temp_xy_opt_path;
% 
% % plot(xy_start(1),xy_start(2),'S',xy_dest(1),xy_dest(2),'D','MarkerSize',10, 'MarkerFaceColor','red')




