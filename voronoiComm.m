function [vxx,vy] = voronoiComm(varargin)
%VORONOI Voronoi diagram.
%   VORONOI(X,Y) plots the Voronoi diagram for the points X,Y. Lines-to-
%   infinity are approximated with an arbitrarily distant endpoint.
%
%   VORONOI(X,Y,OPTIONS) specifies a cell array of strings OPTIONS 
%   that were previously used by Qhull. Qhull-specific OPTIONS are no longer 
%   required and are currently ignored. Support for these options will be 
%   removed in a future release. 
%
%   VORONOI(X,Y,TRI) uses the Delaunay triangulation TRI instead of 
%   computing it internally.
%
%   VORONOI(DelaunayTri) will not accept DelaunayTri in a future release. 
%                   Use VORONOI(delaunayTriangulation) instead.
%   VORONOI(DT) uses the Delaunay triangulation DT instead of computing it 
%   internally, where DT is a DelaunayTri.
%
%   VORONOI(AX,...) plots into AX instead of GCA.
%
%   H = VORONOI(...,'LineSpec') plots the diagram with color and linestyle
%   specified and in H returns two handles; one representing the points and
%   the other representing the lines.
%
%   [VX,VY] = VORONOI(...) returns the vertices of the Voronoi edges in VX 
%   and VY. 
%
%   For the topology of the voronoi diagram, i.e. the vertices for
%   each voronoi cell, use delaunayTriangulation/voronoiDiagram as follows: 
%
%         dt = delaunayTriangulation(X(:),Y(:))
%         [V,C] = voronoiDiagram(dt)
%
%   See also delaunayTriangulation, VORONOIN, DELAUNAY, CONVHULL.

%   Copyright 1984-2015 MathWorks, Inc.


[cax,args,nargs] = axescheck(varargin{:});
if nargs < 1
    error(message('MATLAB:narginchk:notEnoughInputs'));
elseif nargs > 4
    error(message('MATLAB:narginchk:tooManyInputs'));
end

if isa(args{1}, 'DelaunayTri')       
    dt = args{1};
    if dt.size(1) == 0
        error(message('MATLAB:voronoi:EmptyTri'));
    elseif dt.size(2) ~= 3
        error(message('MATLAB:voronoi:NonPlanarTri'));
    end
    x = dt.X(:,1);
    y = dt.X(:,2);
    tri = dt(:,:); 
    if nargs == 1
        ls = '';
    else
        ls = args{2};
    end
elseif isa(args{1}, 'delaunayTriangulation')       
    dt = args{1};
    if dt.size(1) == 0
        error(message('MATLAB:voronoi:EmptyTri'));
    elseif dt.size(2) ~= 3
        error(message('MATLAB:voronoi:NonPlanarTri'));
    end
    x = dt.Points(:,1);
    y = dt.Points(:,2);
    tri = dt(:,:); 
    if nargs == 1
        ls = '';
    else
        ls = args{2};
    end    
else
    x = args{1};
    y = args{2};
    if ~isequal(size(x),size(y))
        error(message('MATLAB:voronoi:InputSizeMismatch'));
    end
    if ndims(x) > 2 || ndims(y) > 2
        error(message('MATLAB:voronoi:HigherDimArray'));
    end
    
    x = x(:);
    y = y(:);   
    if nargs == 2
        tri = delaunay(x,y);
        ls = '';
    else 
        arg3 = args{3};
        if nargs == 3
            ls = '';
        else
            arg4 = args{4};
            ls = arg4;
        end 
        if isempty(arg3)
            tri = delaunay(x,y);
        elseif ischar(arg3)
            tri = delaunay(x,y); 
            ls = arg3;
        elseif iscellstr(arg3)
             error(message('MATLAB:voronoi:DeprecatedOptions'));  
            tri = delaunay(x,y);
        else
            tri = arg3;
        end
    end
end

if isempty(tri)
    return;
end


% Compute centers of triangles
tr = triangulation(tri,x,y);
c = tr.circumcenter();


% Create matrix T where i and j are endpoints of edge of triangle T(i,j)
n = numel(x);
t = repmat((1:size(tri,1))',1,3);
T = sparse(tri,tri(:,[3 1 2]),t,n,n); 

% i and j are endpoints of internal edge in triangle E(i,j)
E = (T & T').*T; 
% i and j are endpoints of external edge in triangle F(i,j)
F = xor(T, T').*T;

% v and vv are triangles that share an edge
[~,~,v] = find(triu(E));
[~,~,vv] = find(triu(E'));

% Internal edges
vx = [c(v,1) c(vv,1)]';
vy = [c(v,2) c(vv,2)]';

%%% Compute lines-to-infinity
% i and j are endpoints of the edges of triangles in z
[i,j,z] = find(F);
% Counter-clockwise components of lines between endpoints
dx = x(j) - x(i);
dy = y(j) - y(i);

% Calculate scaling factor for length of line-to-infinity
% Distance across range of data
rx = max(x)-min(x); 
ry = max(y)-min(y);
% Distance from vertex to center of data
cx = (max(x)+min(x))/2 - c(z,1); 
cy = (max(y)+min(y))/2 - c(z,2);
% Sum of these two distances
nm = sqrt(rx.*rx + ry.*ry) + sqrt(cx.*cx + cy.*cy);
% Compute scaling factor
scale = nm./sqrt((dx.*dx+dy.*dy));
    
% Lines from voronoi vertex to "infinite" endpoint
% We know it's in correct direction because compononents are CCW
ex = [c(z,1) c(z,1)-dy.*scale]';
ey = [c(z,2) c(z,2)+dx.*scale]';
% Combine with internal edges
vx = [vx ex];
vy = [vy ey];

if nargout<2
    % Plot diagram
    if isempty(cax)
        % If no current axes, create one
        cax = gca;
    end
    if isempty(ls)
        % Default linespec
        ls = '-';
    end
    [l,c,mp,msg] = colstyle(ls); error(msg) % Extract from linespec
    if isempty(mp)
        % Default markers at points        
        mp = '.';
    end
     if isempty(l)
        % Default linestyle
        l = get(ancestor(cax,'figure'),'DefaultAxesLineStyleOrder'); 
    end
    if isempty(c), 
        % Default color        
        co = get(ancestor(cax,'figure'),'DefaultAxesColorOrder');
        c = co(1,:);
    end
    % Plot points
    h1 = plot(x,y,'marker',mp,'color',c,'linestyle','none','parent',cax);
    % Plot voronoi lines
    nume = size(vx,2);
    vx = [vx; NaN(1,nume)];
    vx = vx(:);
    vy = [vy; NaN(1,nume)];
    vy = vy(:);
    h2 = line(vx,vy,'color',c,'linestyle',l,'parent',cax,...
        'yliminclude','off','xliminclude','off');
    if nargout==1, vxx = [h1; h2]; end % Return handles
else
    vxx = vx; % Don't plot, just return vertices
end




