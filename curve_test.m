clear;
clc;

seta = 0 : 0.01 : 2*pi;
seta1 = pi/6;
seta2 = pi/10;
seta3 = -pi/3;

x = [2 7 -6];
y = [1 -2 -8];

% x1 = 2*cos(seta1);
% y1 = 2*sin(seta1);
% 
% x2 = 5*cos(seta2);
% y2 = 5*sin(seta2);
% 
% x3 = 3*cos(seta3);
% y3 = 3*sin(seta3);


figure(1)
plot(x, y, 'xk', 'MarkerSize', 10);
% plot([x1 x2 x3], [y1 y2 y3], 'xk', 'MarkerSize', 10);
axis([-10 10 -10 10]);
grid on;
hold on;

temp_xy_opt_path2 = [x' y'];
% temp_xy_opt_path2 = [x1 y1; x2 y2; x3 y3];



%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
p = 2;

% while p < length(temp_xy_opt_path2)-2

    % 동일 좌표로 인식 할 경우 스킵
%     if(temp_xy_opt_path2(p,:) == temp_xy_opt_path2(p+1,:))
%         continue;
%     end

    if length(temp_xy_opt_path2) > 1      % 원을 그릴 수 있는 최소 지점의 개수

        x1 = (temp_xy_opt_path2(1,1)+temp_xy_opt_path2(2,1))/2;y1 = (temp_xy_opt_path2(1,2)+temp_xy_opt_path2(2,2))/2;   
        x2 = temp_xy_opt_path2(2,1); y2 = temp_xy_opt_path2(2,2) ; 
        x3 = (temp_xy_opt_path2(3,1)+temp_xy_opt_path2(2,1))/2; y3  = (temp_xy_opt_path2(3,2)+temp_xy_opt_path2(2,2))/2; 

        a = [x1, y1, 1;    x2, y2, 1;    x3, y3, 1];
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

%                     tempangle = useta(2)/180*pi:0.01:max(useta)/180*pi;
%                     tempangle2 = 0:0.01:min(useta)/180*pi;
%                     thtemp = [tempangle,tempangle2];


        else
            thtemp = min(useta)/180*pi:0.01:max(useta)/180*pi; %원을 그리기위한 각도 배열
        end

        %thtemp = useta(3)/180*pi:0.01:useta(1)/180*pi; %원을 그리기위한 각도 배열
        c_cxt = centerx; c_cyt = centery; c_rt =radius;

%         if(( (x1 - c_cxt)^2 + (y1 - c_cyt)^2 ) ~= c_rt^2)
%             if (( (x2 - c_cxt)^2 + (y2 - c_cyt)^2 ) ~= c_rt^2)
%                 disp("x1, y1 다르다");
%                 plot([x1 x2], [y1 y2], 'red','LineWidth',2);
% 
%             end
% 
%         elseif (( (x2 - c_cxt)^2 + (y2 - c_cyt)^2 ) ~= c_rt^2)
%             disp("x2 다르다");
% 
%         elseif (( (x3 - c_cxt)^2 + (y3 - c_cyt)^2 ) ~= c_rt^2)
%             disp("x3 다르다");    
% 
%         end

        xct = c_rt*cos(thtemp)+c_cxt; 
        yct = c_rt*sin(thtemp)+c_cyt;

        % 동일 직선상에 있을 경우 원 X
        if (((x1 == x2) && (x2 == x3)) || ((y1 == y2) && (y2 == y3)))
            plot([x1 x2 x3], [y1 y2 y3], 'red','LineWidth',2);

        end

        plot(xct,yct,'red','LineWidth',2);                   


    end % if end

%     p=p+1;  

% end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%















