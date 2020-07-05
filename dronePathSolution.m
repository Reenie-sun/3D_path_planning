function [drone_count, path_value] = dronePathSolution(xpoint, ypoint, drone_v, drone_t)

temp_xpoint = xpoint;
temp_ypoint = ypoint;

drone_count = 0;

b_temp_x = [];
b_temp_y = [];

drone_s = drone_v * drone_t;

while (length(temp_xpoint) > 2)
    
    path_all = 0;
    
    if(length(temp_xpoint) < 3)
        plot(temp_xpoint, temp_ypoint, 'LineWidth', 3);
        
    else
    
        temp_order = tsp(temp_xpoint,temp_ypoint);

        for i = 1 : length(temp_xpoint)-1
            path_all = path_all + sqrt( (temp_xpoint(temp_order(i)) - temp_xpoint(temp_order(i+1)))^2 + (temp_ypoint(temp_order(i)) - temp_ypoint(temp_order(i+1)))^2 );
        end

        if(path_all < drone_s)
            plot(temp_xpoint(temp_order), temp_ypoint(temp_order), 'LineWidth', 3);
                     
            drone_count = drone_count + 1;
            
            path_value(drone_count) = path_all;

            temp_xpoint = b_temp_x;
            temp_ypoint = b_temp_y;
            
            b_temp_x = [];
            b_temp_y = [];

        else

            half_order = floor(length(temp_order)/2);

            porder = temp_order(1:half_order);
            border = temp_order(half_order+1:end);

            path1 = sqrt( (temp_xpoint(porder(end)) - temp_xpoint(porder(end-1)))^2 + (temp_ypoint(porder(end)) - temp_ypoint(porder(end-1)))^2 );
            path2 = sqrt( (temp_xpoint(porder(end)) - temp_xpoint(border(1)))^2 + (temp_ypoint(porder(end)) - temp_ypoint(border(1)))^2 );

            if path1 > path2
                border(end+1) = porder(end);
                porder(end) = [];

            end

            border = sort(border);
                       
            b_temp_x = temp_xpoint(border);
            b_temp_y = temp_ypoint(border);
            
            temp_xpoint = temp_xpoint(porder);
            temp_ypoint = temp_ypoint(porder);

        end
        
        
    end
        

end