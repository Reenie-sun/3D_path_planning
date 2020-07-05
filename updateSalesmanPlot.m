function lh = updateSalesmanPlot(lh,xopt,idxs,stopsLat,stopsLon)
% Plotting function for tsp_intlinprog example

%   Copyright 2014-2016 The MathWorks, Inc. 

if ( lh ~= zeros(size(lh)) ) % First time through lh is all zeros
    set(lh,'Visible','off'); % Remove previous lines from plot
end

% original
segments = find(round(xopt)); % Indices to trips in solution
% segments = find(ceil(xopt)); % Indices to trips in solution

% Loop through the trips then draw them
Lat = zeros(3*length(segments),1);
Lon = zeros(3*length(segments),1);
for ii = 1:length(segments)
    start = idxs(segments(ii),1);
    stop = idxs(segments(ii),2);
    
    % Separate data points with NaN's to plot separate line segments
    Lat(3*ii-2:3*ii) = [stopsLat(start); stopsLat(stop); NaN];
    Lon(3*ii-2:3*ii) = [stopsLon(start); stopsLon(stop); NaN];  
end

lh = plot(Lat,Lon,'k:','LineWidth',2 ,'Visible','off');

set(lh,'Visible','off'); %drawnow; % Add new lines to plot