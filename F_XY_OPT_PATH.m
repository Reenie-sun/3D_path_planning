function xy_opt_path = F_XY_OPT_PATH(xy_start, xy_dest, vx_new, vy_new, xn_new, yn_new, xc, yc)

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

%1ø≠¿« µ•¿Ã≈Õ »πµÊ
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