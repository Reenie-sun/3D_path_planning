clear;


[Z, ref] = dted('n37.dt0');

N_s = 30;
x_s = linspace(0, 30,N_s);
y_s = linspace(0, 30,N_s);

x_2d_s = repmat(x_s,N_s,1);
y_2d_s = repmat(y_s',1,N_s);

x_1d_s = zeros(1,30*30);
y_1d_s = zeros(1,30*30);
z_1d_s = zeros(1,30*30);
z_2d_s = zeros(30,30);

cnt = 1;
for i = 1: 30 %1자로 만드는부분
    for j = 1: 30
        x_1d_s(cnt) = x_2d_s(i,j);
        y_1d_s(cnt) = y_2d_s(i,j);
        z_1d_s(cnt) = Z(i,j);
        z_2d_s(i,j) = Z(i,j);
        cnt = cnt+1;
    end
end

i_site = find(z_1d_s > 400);  %높이 값주는부분 실제로 사용될 크기 판단
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

xy_start = [25 6];
xy_dest = [1.5 17];

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


figure(5);
set(gcf,'numbertitle','off','name', '최종경로');
plot(xy_start(1),xy_start(2),'S',xy_dest(1),xy_dest(2),'D','MarkerSize',10, 'MarkerFaceColor','red')
hold on;
contour(x_2d_s,y_2d_s,z_2d_s,'ShowText','on');
xlabel('km');
ylabel('km');

plot(vx_new,vy_new,'b.-')
plot(xn_new,yn_new,'r.');
h = fill(xc,yc,'red');
set(h,'facealpha',.5)
axis([0 30 0 30]);
plot(xy_opt_path(:,1),xy_opt_path(:,2),'ko-','LineWidth',2);

p = 2;

while p < length(xy_opt_path)-2
    if length(xy_opt_path) > 5
        x1 = (xy_opt_path(p,1)+xy_opt_path(p+1,1))/2;y1 = (xy_opt_path(p,2)+xy_opt_path(p+1,2))/2;   
        x2 = xy_opt_path(p+1,1); y2 = xy_opt_path(p+1,2) ; 
        x3 = (xy_opt_path(p+2,1)+xy_opt_path(p+1,1))/2; y3  = (xy_opt_path(p+2,2)+xy_opt_path(p+1,2))/2; 

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
       
        
        resulty = (((y3-y1)/(x3-x1)) * (x2-x1)) + y1;
        
        if x1 > x3 %시작점이 오쪽에 있을때
            if y1 > y3 %시작점이 오위 y값이 크면 왼쪽에있는거
                if resulty > y2 %오른쪽에 있네
                    if seta1 < 180
                        tempangle1 = 0 : 0.01 : seta1/180*pi;
                        tempangle2 = seta3/180*pi : 0.01 : 2*pi;
                        thtemp = [tempangle2,tempangle1];
                    else
                        thtemp = seta3/180*pi : 0.01 : seta1/180*pi;
                    end
                    
                else %왼쪽에 있네
                    if seta1 > 180
                         tempangle1 = seta1/180*pi : 0.01 : 2*pi;
                         tempangle2 = 0 : 0.01 : seta3;
                         thtemp = [tempangle1,tempangle2];
                    else
                        thtemp = seta1/180*pi : 0.01 : seta3/180*pi;
                    end
                end
            elseif y1 < y3 %시작점이 오아 y값이 크면 오쪽에있는거
                if resulty > y2 %왼쪽에 있네
                    if seta1 < 180
                        tempangle1 = 0 : 0.01 : seta1/180*pi;
                        tempangle2 = seta3/180*pi : 0.01 : 2*pi;
                        thtemp = [tempangle1,tempangle2];
                    else
                        thtemp = seta3/180*pi : 0.01 : seta1/180*pi;
                    end
                else %오쪽에 있네
                    if seta1 < 180
                        thtemp = seta1/180*pi : 0.01 : seta3/180*pi;
                    else
                        tempangle1 = seta1/180*pi : 0.01 : (2*pi)-0.01;
                        tempangle2 = 0 : 0.01 : seta3/180*pi;
                        thtemp = [tempangle1,tempangle2];
                    end
                end
            else
                thtemp = seta3/180*pi : 0.01 : seta1/180*pi;
            end
        elseif x1 < x3 % 시작점이 왼쪽에 있을때
            if y1 > y3 %시작점이 왼위 y값이 크면 오쪽에있는거
                if resulty > y2 %왼쪽에 있네
                    if seta3 < 180
                        tempangle1 = seta1/180*pi : 0.01 : 2*pi;
                        tempangle2 = 0 : 0.01 : seta3/180*pi;
                        thtemp = [tempangle1,tempangle2];
                    else
                        thtemp = seta1/180*pi : 0.01 : seta3/180*pi;
                    end
                        
                else %오쪽에 있네
                    if seta3 <180
                        thtemp = seta3/180*pi : 0.01 : seta1/180*pi;
                    else
                        tempangle1 = 0 : 0.01 : seta1/180*pi;
                        tempangle2 = seta3/180*pi : 0.01 : 2*pi;
                        thtemp = [tempangle2,tempangle1];
                    end
                end
            elseif y1 < y3 %시작점이 왼아 y값이 크면 왼쪽에있는거
                if resulty > y2 %오른쪽에 있네
                    if seta3 < 180
                        tempangle1 = seta1/180*pi : 0.01 : 2*pi;
                        tempangle2 = 0 : 0.01 : seta3/180*pi;
                        thtemp = [tempangle1,tempangle2];
                    else
                         thtemp = seta1/180*pi : 0.01 : seta3/180*pi;
                    end
                   
                else %왼쪽에 있네
                    if seta3 < 180
                        thtemp = seta3/180*pi : 0.01 : seta1/180*pi;
                    else
                        tempangle1 = 0 : 0.01 : seta1/180*pi;
                        tempangle2 = seta3/180*pi : 0.01 : 2*pi;
                        thtemp = [tempangle1,tempangle2];
                    end
                end
            else
                thtemp = seta1/180*pi : 0.01 : seta3/180*pi;
            end      
         end
        
        c_cxt = centerx; c_cyt = centery; c_rt =radius;
        xct = c_rt*cos(thtemp)+c_cxt; 
        yct = c_rt*sin(thtemp)+c_cyt;
        plot(xct,yct,'red','LineWidth',4);
        p=p+1;
        
    end
end


