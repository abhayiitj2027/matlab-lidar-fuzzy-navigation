function path=visibility_route(start,goal,obs,margin)
% Dijkstra over a finite visibility graph of inflated measured disks.
nodes=[start;goal];radii=obs(:,4)+.6+margin;
for j=1:size(obs,1)
    rr=(radii(j)+.02)/cos(pi/16);
    for a=(0:15)*2*pi/16
        q=obs(j,2:3)+rr*[cos(a) sin(a)];
        if clear_segment(q,q,obs,radii),nodes(end+1,:)=q;end %#ok<AGROW>
    end
end
n=size(nodes,1);adj=inf(n);adj(1:n+1:end)=0;
for i=1:n
    for j=1:i-1
        if clear_segment(nodes(i,:),nodes(j,:),obs,radii)
            adj(i,j)=norm(nodes(i,:)-nodes(j,:));adj(j,i)=adj(i,j);
        end
    end
end
dist=inf(1,n);dist(1)=0;previous=zeros(1,n);visited=false(1,n);
for iteration=1:n
    candidate=dist;candidate(visited)=inf;[cost,i]=min(candidate);
    if ~isfinite(cost)||i==2,break;end
    visited(i)=true;
    for j=1:n
        if ~visited(j)&&cost+adj(i,j)<dist(j)
            dist(j)=cost+adj(i,j);previous(j)=i;
        end
    end
end
path=zeros(0,2);if ~isfinite(dist(2)),return;end
indices=2;while indices(1)~=1,indices=[previous(indices(1)) indices];end %#ok<AGROW>
for k=1:numel(indices)-1
    a=nodes(indices(k),:);b=nodes(indices(k+1),:);count=max(1,ceil(norm(b-a)/.15));
    path=[path;a+(0:count-1)'/count*(b-a)]; %#ok<AGROW>
end
path=[path;goal];
end
function free=clear_segment(a,b,obs,radii)
if isempty(obs),free=true;return;end
delta=b-a;den=sum(delta.^2);
if den==0,t=zeros(size(obs,1),1);else,t=max(0,min(1,(obs(:,2:3)-a)*delta'/den));end
free=all(vecnorm(a+t*delta-obs(:,2:3),2,2)>=radii-1e-8);
end
