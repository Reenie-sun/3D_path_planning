function [drone_x, drone_count, path_value] = dronePathSolution_cell(xpoint, ypoint, drone_v, drone_t, vx_new, vy_new, xn_new, yn_new, xc, yc)

%%%%%%%%%%%%%%%%%% function 내부

    drone_s = d_time * d_velo;

    drone_count = 0;
    path_value = 0; 

    b_temp_x = [];
    b_temp_y = [];

    temp_xpoint = xpoint;
    temp_ypoint = ypoint;

    drone_x = 0;
    
    path_part_all = [];
    
    temp_xy_opt = [];


    while (length(temp_xpoint) >= 2)

        path_all = 0;

        if(length(temp_xpoint) < 3)

            temp_order = [1 2];
            
            %%%%%%%%%%%%%%%%%%%%%%%     경로 계산
            xy_start = [temp_xpoint(temp_order(1)) temp_ypoint(temp_order(1))];
            xy_dest = [temp_xpoint(temp_order(2)) temp_ypoint(temp_order(2))];

            xy_opt_path = F_XY_OPT_PATH(xy_start, xy_dest, vx_new, vy_new, xn_new, yn_new, xc, yc);

            % 전체 경로 길이 구간별 계산
            path_all = 0;

            for i = 1 : length(xy_opt_path)-1
                path_all = sqrt((xy_opt_path(i,1) - xy_opt_path(i+1,1))^2 + (xy_opt_path(i,2) - xy_opt_path(i+1,2))^2) + path_all;

            end

%             tpath = path_all;
%             tpath
            
            path_all = 2 * path_all;
            
            

            %%%%%%%%%%%%%%%%%%%%%%

            if(path_all < drone_s)

        %             disp('한줄 그렸다');
        
                % 사용자가 버튼을 누를때까지 대기
                waitforbuttonpress;

                plot(xy_opt_path(:,1), xy_opt_path(:,2), 'LineWidth', 3);

                drone_count = drone_count + 1;

                path_value(drone_count) = path_all;
                
                path_part_all{drone_count} = path_all;
                
                xy_count = xy_count + 1;
                
                temp_xy_opt_path{end+1} = xy_opt_path;

%                 set(handles.edit_drone_now_way, 'String', sprintf('%s [km]', num2str(path_all)));
%                 set(handles.edit_drone_num, 'String', sprintf('%d [대]', drone_count));

            else
                disp('경로의 길이가 부족합니다.');
                path_lack = path_all;
                path_lack

                drone_x = drone_x + 1;

            end

            temp_xpoint = b_temp_x;
            temp_ypoint = b_temp_y;

            b_temp_x = [];
            b_temp_y = [];        

        else

            temp_order = tsp(temp_xpoint,temp_ypoint);
            
            temp_order

            tpath = 0;
            xy_opt = [];
            path_part = [];
            
            temp_xy = [];

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%       경로 계산
            for mainloof = 1: length(temp_xpoint)

                % 마지막 부분
                if(mainloof == length(temp_xpoint))
                    %%%%%%%%%%%%%

                    xy_start = [temp_xpoint(temp_order(end-1)) temp_ypoint(temp_order(end-1))];
                    xy_dest = [temp_xpoint(temp_order(1)) temp_ypoint(temp_order(1))];

                    %%%%%%%%%%%%% 

                else

                    %%%%%%%%%%%%%

                    xy_start = [temp_xpoint(temp_order(mainloof)) temp_ypoint(temp_order(mainloof))];
                    xy_dest = [temp_xpoint(temp_order(mainloof+1)) temp_ypoint(temp_order(mainloof+1))];

                    %%%%%%%%%%%%%

                end

                xy_opt_path = F_XY_OPT_PATH(xy_start, xy_dest, vx_new, vy_new, xn_new, yn_new, xc, yc);

                % 전체 경로 길이 구간별 계산
                tpath = 0;

                for i = 1 : length(xy_opt_path)-1
                    tpath = sqrt((xy_opt_path(i,1) - xy_opt_path(i+1,1))^2 + (xy_opt_path(i,2) - xy_opt_path(i+1,2))^2) + tpath;

                end

%                 tpath
                
                path_all = path_all + tpath;

                path_part(mainloof) = tpath;

                temp_xy{mainloof} = xy_opt_path;
                xy_count = xy_count + 1;

            end % for mainloof = 1: length(temp_order)-1 end

            %%%%%%%%%%%%%%%%%%%%%%%%%%%     경로 계산 끝   

            if(path_all < drone_s)
                
                % 사용자가 버튼을 누를때까지 대기
                waitforbuttonpress;

        %             disp('그렸다');

                xy_opt = temp_xy{1};

                for jj = 2 : length(temp_xpoint)
                    test = temp_xy{jj};
                    xy_opt = [xy_opt ; test];

                end

                plot(xy_opt(:,1), xy_opt(:,2), 'LineWidth', 3);

        %             path_all

                drone_count = drone_count + 1;

                path_value(drone_count) = path_all;
                
                path_part_all{drone_count} = path_part;
                
                for ii = 1 : length(temp_xy)
                
                    temp = temp_xy{ii};
                    temp_xy_opt_path{end+1} = temp;
                    
                end

                temp_xpoint = b_temp_x;
                temp_ypoint = b_temp_y;

                b_temp_x = [];
                b_temp_y = [];

%                 set(handles.edit_drone_now_way, 'String', sprintf('%s [km]', num2str(path_all)));
%                 set(handles.edit_drone_num, 'String', sprintf('%d [대]', drone_count));

            else

                half_order = ceil(length(temp_order)/2);

                porder = temp_order(1:half_order);
                border = temp_order(half_order+1:end);

                path1 = 0;
                path2 = 0;

                xy_opt_path1 = temp_xy{half_order-1};
                xy_opt_path2 = temp_xy{half_order};

                if (length(porder) > 2)

                    for i = 1 : length(xy_opt_path1)-1
                        path1 = sqrt((xy_opt_path1(i,1) - xy_opt_path1(i+1,1))^2 + (xy_opt_path1(i,2) - xy_opt_path1(i+1,2))^2) + path1;
                    end

                    for i = 1 : length(xy_opt_path2)-1
                        path2 = sqrt((xy_opt_path2(i,1) - xy_opt_path2(i+1,1))^2 + (xy_opt_path2(i,2) - xy_opt_path2(i+1,2))^2) + path2;
                    end

                    if path1 > path2
                        border(end+1) = porder(end);
                        porder(end) = [];

                    end

                end

                border = sort(border);

                % 비어있으면 1
                if(isempty(b_temp_x) == 1)
                    b_temp_x = temp_xpoint(border);
                    b_temp_y = temp_ypoint(border);
                else
        %                 b_temp_x = cat(2, b_temp_x, temp_xpoint(border(2:end)));
        %                 b_temp_y

                    b_temp_x = [b_temp_x' temp_xpoint(border(2:end))'];
                    b_temp_y = [b_temp_y' temp_ypoint(border(2:end))'];

                end


                temp_xpoint = temp_xpoint(porder);
                temp_ypoint = temp_ypoint(porder);

            end


        end


    end
    
    assignin('base', 'path_part_all', path_part_all);
    assignin('base', 'temp_xy_opt_path', temp_xy_opt_path); 
