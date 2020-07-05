function order = tsp(givenx,giveny)


nStops = length(givenx); % you can use any number, but the problem size scales as N^2
stopsLon = zeros(length(givenx),1); % allocate x-coordinates of nStops
stopsLat = stopsLon; % allocate y-coordinates

stopsLat = giveny;
stopsLon = givenx;

%plot(x,y,'Color','red'); % draw the outside border
%hold on
% Add the stops to the map
%plot(stopsLon,stopsLat,'*b')
%hold off


idxs = nchoosek(1:nStops,2);

dist = hypot(stopsLat(idxs(:,1)) - stopsLat(idxs(:,2)), ...
             stopsLon(idxs(:,1)) - stopsLon(idxs(:,2)));
lendist = length(dist);

Aeq = spones(1:length(idxs)); % Adds up the number of trips
beq = nStops;

Aeq = [Aeq;spalloc(nStops,length(idxs),nStops*(nStops-1))]; % allocate a sparse matrix
for ii = 1:nStops
    whichIdxs = (idxs == ii); % find the trips that include stop ii
    whichIdxs = sparse(sum(whichIdxs,2)); % include trips where ii is at either end
    Aeq(ii+1,:) = whichIdxs'; % include in the constraint matrix
end
beq = [beq; 2*ones(nStops,1)];                                                                                                                                 








intcon = 1:lendist;
lb = zeros(lendist,1);
ub = ones(lendist,1);

opts = optimoptions('intlinprog','Display','off','Heuristics','advanced');
[x_tsp,costopt,exitflag,output] = intlinprog(dist,intcon,[],[],Aeq,beq,lb,ub,opts);

%hold on
segments = find(x_tsp); % Get indices of lines on optimal path
lh = zeros(nStops,1); % Use to store handles to lines on plot
lh = updateSalesmanPlot(lh,x_tsp,idxs,stopsLon,stopsLat);
%title('Solution with Subtours');

tours = detectSubtours(x_tsp,idxs);
numtours = length(tours); % number of subtours
%fprintf('# of subtours: %d\n',numtours);

A = spalloc(0,lendist,0); % Allocate a sparse linear inequality constraint matrix
b = [];
while numtours > 1 % repeat until there is just one subtour
    % Add the subtour constraints
    b = [b;zeros(numtours,1)]; % allocate b
    A = [A;spalloc(numtours,lendist,nStops)]; % a guess at how many nonzeros to allocate
    for ii = 1:numtours
        rowIdx = size(A,1)+1; % Counter for indexing
        subTourIdx = tours{ii}; % Extract the current subtour
%         The next lines find all of the variables associated with the
%         particular subtour, then add an inequality constraint to prohibit
%         that subtour and all subtours that use those stops.
        variations = nchoosek(1:length(subTourIdx),2);
        for jj = 1:length(variations)
            whichVar = (sum(idxs==subTourIdx(variations(jj,1)),2)) & ...
                       (sum(idxs==subTourIdx(variations(jj,2)),2));
            A(rowIdx,whichVar) = 1;
        end
        b(rowIdx) = length(subTourIdx)-1; % One less trip than subtour stops
    end

    % Try to optimize again
    [x_tsp,costopt,exitflag,output] = intlinprog(dist,intcon,A,b,Aeq,beq,lb,ub,opts);
    
    % Visualize result
    lh = updateSalesmanPlot(lh,x_tsp,idxs,stopsLon,stopsLat);
    
    % How many subtours this time?
    tours = detectSubtours(x_tsp,idxs);
    numtours = length(tours); % number of subtours
    %fprintf('# of subtours: %d\n',numtours);
end
x_site = lh.XData;
y_site = lh.YData;


order = zeros(length(givenx)+1,1);
order(1) = find(x_site(1)==stopsLon & y_site(1)==stopsLat);
order(2) =find(x_site(2)==stopsLon & y_site(2)==stopsLat);

for iiiii = 1 : 3
    x_site(1) = [];
    y_site(1) = [];
end

for myi = 3 : length(order)
   temp = find(x_site == stopsLon(order(myi-1)) & y_site == stopsLat(order(myi-1)));
   if(~isnan(x_site(temp(1)-1)))
       order(myi) = find(x_site(temp(1)-1) == stopsLon & y_site(temp(1)-1)==stopsLat);
       for fori1 = 1 : 3
            x_site(temp-1) = [];
            y_site(temp-1) = [];
        end
   else
       order(myi) = find(x_site(temp(1)+1) == stopsLon & y_site(temp(1)+1)==stopsLat);
       for fori1 = 1 : 3
            x_site(temp) = [];
            y_site(temp) = [];
        end
   end
%    stopsLon
%    stopsLat
%    x_site
%    y_site
%    order
  
end

%order(length(order)+1) = 1;
%close all force;
%title('Solution with Subtours Eliminated');
%hold off
%disp(output.absolutegap)