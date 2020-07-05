function [th] = findLiftAngle(x_obs, y_obs, z_obs, count_obs)

th = [];

% count_obs = 3;
% 
% x_obs = [6 8 11 13 14 15 16];
% y_obs = [6 7 10 11 12 13 13];
% z_obs = [10 11 14 12 9 11 14];

k = 1;

% 장애물 개수만큼 검사 진행
for j = 1 : count_obs
    for i = 1 : length(z_obs)-1

        % 높이 상승
        if(z_obs(i+1) > z_obs(i))
            a = ( (y_obs(i+1) - y_obs(i)) / (x_obs(i+1) - x_obs(i)) );
            th_temp = atan(a);
            % radian -> degree
            th(k) = th_temp * 180 / pi;
            k = k + 1;
            
%             display('hello');

            break;
        end

    end
end