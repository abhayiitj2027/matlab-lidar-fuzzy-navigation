function result=simulate_lidar(maxTime)
% SIMULATE_LIDAR A native MATLAB reimplementation, not Python data playback.
% World: [x y radius vx vy isMoving]. Navigation never receives this array.
if nargin<1, maxTime=120; end
rng(7,'twister'); dt=.05; truth=[0 0 0]; actual=[0 0]; pose=truth;
world=[3 .1 .35 0 0 0;6 -.2 .35 0 0 0;1.7 2.3 .32 0 0 0; ...
    8 2.3 .36 0 0 0;4.8 2 .30 0 -.25 1;7.2 -2.7 .28 0 .07 1];
tracks=zeros(0,8); nextId=1; obs=zeros(0,7); landmarks=zeros(0,2);
map=zeros(81,101); observed=false(size(map)); res=.15; origin=[-3 -6];
path=zeros(0,2); trail=zeros(0,2); frames={}; telemetry=zeros(0,9);
ctrl=struct('weights',zeros(1,7),'basis',[],'lastW',0);
replans=0; corrections=0; voTotal=0; lastSignature=zeros(0,2);
status='Running'; goal=[9 0]; basis=zeros(1,7); feedback=[0 0];
for step=0:ceil(maxTime/dt)-1
    now=step*dt;
    if step>0
        feedback=actual.*[1.015 1.01]+[.002 .001].*randn(1,2);
        pose=pose+[feedback(1)*cos(pose(3))*dt feedback(1)*sin(pose(3))*dt feedback(2)*dt];
        pose(3)=wrap(pose(3));
    end
    % Only the sensor generator, plant and evaluator may access world truth.
    ranges=lidar_scan(truth,world);
    local=features(ranges,[0 0 0]);
    oldPose=pose; pairs=zeros(0,4); used=false(size(landmarks,1),1);
    for j=1:size(local,1)
        q=transform(local(j,1:2),pose);
        moving=obs(obs(:,7)>0,:);
        if ~isempty(moving) && any(vecnorm(q-(moving(:,2:3)+moving(:,5:6)*dt),2,2)<moving(:,4)+.25), continue; end
        distances=vecnorm(landmarks-q,2,2); distances(used)=inf;
        [v,idx]=min(distances);
        if ~isempty(v) && v<.20
            pairs(end+1,:)=[local(j,1:2) landmarks(idx,:)]; used(idx)=true; %#ok<AGROW>
        end
    end
    if ~isempty(pairs)
        for delta=[.02 .006 .002]
            best=inf; chosen=pose; angles=0;
            if size(pairs,1)>=2, angles=[-delta/3 0 delta/3]; end
            for dx=[-delta 0 delta]
                for dy=[-delta 0 delta]
                    for da=angles
                        p=pose+[dx dy da]; q=transform(pairs(:,1:2),p);
                        cost=sum(min(.04,sum((q-pairs(:,3:4)).^2,2)))+.001*sum((p(1:2)-oldPose(1:2)).^2);
                        if cost<best, best=cost; chosen=p; end
                    end
                end
            end
            pose=chosen;
        end
        corrections=corrections+(norm(pose-oldPose)>1e-6);
    end
    [found,points]=features(ranges,pose);
    [tracks,nextId,obs]=track_objects(found,tracks,nextId,now);
    for j=1:size(tracks,1)
        if tracks(j,8)>=12 && norm(tracks(j,5:6))<=.025 && ...
                (isempty(landmarks)||all(vecnorm(landmarks-tracks(j,2:3),2,2)>=.5))
            landmarks(end+1,:)=tracks(j,2:3); %#ok<AGROW>
        end
    end
    [map,observed]=update_map(map,observed,ranges,pose,obs,res,origin);
    signature=obs(:,[1 7]);
    if mod(step,5)==0 || ~isequal(signature,lastSignature)
        expanded=obs;
        for j=1:size(obs,1)
            if obs(j,7)
                for future=[.75 1.5 2.25]
                    o=obs(j,:);o(2:3)=o(2:3)+o(5:6)*future;o(4)=o(4)+.03*future;
                    expanded(end+1,:)=o; %#ok<AGROW>
                end
            end
        end
        path=visibility_route(pose(1:2),goal,expanded,.25);
        if isempty(path), path=visibility_route(pose(1:2),goal,obs,.05); end
        replans=replans+1;
    end
    lastSignature=signature;
    [command,ctrl,status,basis,stats]=fuzzy_navigation(pose,path,obs,feedback,ctrl,goal);
    voTotal=voTotal+stats.voRejected;
    actual=actual+max(-[.6 1.5]*dt,min([.6 1.5]*dt,command-actual));
    truth=truth+[actual(1)*cos(truth(3))*dt actual(1)*sin(truth(3))*dt actual(2)*dt];
    truth(3)=wrap(truth(3));
    world(:,1:2)=world(:,1:2)+world(:,4:5)*dt;
    for j=1:size(world,1)
        if world(j,6)
            if world(j,2)>2.7, world(j,2)=2.7;world(j,5)=-abs(world(j,5)); end
            if world(j,2)<-2.7, world(j,2)=-2.7;world(j,5)=abs(world(j,5)); end
        end
    end
    clearance=min(vecnorm(world(:,1:2)-truth(1:2),2,2)-world(:,3)-.6);
    reached=norm(pose(1:2)-goal)<.25; collision=clearance<0;
    if collision, status='COLLISION DETECTED'; elseif reached, status='GOAL REACHED'; end
    trail(end+1,:)=pose(1:2); %#ok<AGROW>
    telemetry(end+1,:)=[now+dt truth(1:2) pose(1:2) command clearance replans]; %#ok<AGROW>
    if mod(step+1,2)==0 || reached || collision
        f=struct('t',now+dt,'pose',pose,'world',world,'obs',obs,'points',points, ...
            'path',path,'trail',trail,'map',map,'observed',observed,'origin',origin,'resolution',res, ...
            'command',command,'basis',basis,'status',status,'replans',replans, ...
            'landmarks',size(landmarks,1),'corrections',corrections,'voTotal',voTotal);
        frames{end+1}=f; %#ok<AGROW>
    end
    if mod(step,100)==0, fprintf('MATLAB simulation %.1f s, %s\n',now,status); end
    if reached||collision, break; end
end
if ~reached && ~collision, status='TIME LIMIT REACHED';frames{end}.status=status; end
summary=struct('status',status,'success',reached && ~collision,'simulated_seconds',now+dt, ...
    'minimum_true_clearance',min(telemetry(:,8)),'replans',replans,'vo_rejections',voTotal, ...
    'landmarks',size(landmarks,1),'pose_corrections',corrections,'runtime','MATLAB', ...
    'scope','Bare planar robot; ideal LiDAR; educational local SLAM; no payload or MPC');
result=struct('summary',summary,'frames',{frames},'telemetry',telemetry);
end

function ranges=lidar_scan(p,world)
a=p(3)-pi+(0:359)'*2*pi/360; rays=[cos(a) sin(a)]; ranges=inf(360,1);
for j=1:size(world,1)
    delta=world(j,1:2)-p(1:2);along=rays*delta';disc=world(j,3)^2-sum(delta.^2)+along.^2;
    d=along-sqrt(max(0,disc));valid=disc>=0 & d>=.05 & d<=3.5;
    ranges(valid)=min(ranges(valid),d(valid));
end
end

function [found,points]=features(ranges,pose)
points=zeros(0,2);groups={};current=zeros(0,2);last=-2;
for i=1:numel(ranges)
    d=ranges(i);
    if ~isfinite(d)||d<.05||d>3.5
        if ~isempty(current),groups{end+1}=current;current=zeros(0,2);end %#ok<AGROW>
        continue
    end
    a=pose(3)-pi+(i-1)*2*pi/360;pt=pose(1:2)+d*[cos(a) sin(a)];points(end+1,:)=pt; %#ok<AGROW>
    if ~isempty(current)&&(i~=last+1||norm(pt-current(end,:))>.22)
        groups{end+1}=current;current=zeros(0,2); %#ok<AGROW>
    end
    current(end+1,:)=pt;last=i; %#ok<AGROW>
end
if ~isempty(current),groups{end+1}=current;end
if numel(groups)>1&&norm(groups{1}(1,:)-groups{end}(end,:))<.22
    groups{1}=[groups{end};groups{1}];groups(end)=[];
end
found=zeros(0,3);
for i=1:numel(groups)
    pts=groups{i};center=mean(pts,1);radius=max(vecnorm(pts-center,2,2))+.15;
    if size(pts,1)>=5
        a=pts(1,:);b=pts(floor(size(pts,1)/2)+1,:)-a;c=pts(end,:)-a;det=2*(b(1)*c(2)-b(2)*c(1));
        if abs(det)>1e-6
            u=[(sum(b.^2)*c(2)-sum(c.^2)*b(2))/det (b(1)*sum(c.^2)-c(1)*sum(b.^2))/det];
            r=norm(u); q=a+u;
            if r>=.05&&r<=1.2&&max(abs(vecnorm(pts-q,2,2)-r))<.035,center=q;radius=r+.04;end
        end
    end
    found(end+1,:)=[center radius]; %#ok<AGROW>
end
end

function [tracks,nextId,obs]=track_objects(found,tracks,nextId,now)
available=true(size(tracks,1),1);
for i=1:size(found,1)
    predicted=tracks(:,2:3)+tracks(:,5:6).*(now-tracks(:,7));
    distances=vecnorm(predicted-found(i,1:2),2,2);distances(~available)=inf;[dist,j]=min(distances);
    velocity=[0 0];hits=1;
    if ~isempty(dist)&&dist<.55
        old=tracks(j,:);elapsed=now-old(7);velocity=old(5:6);hits=old(8)+1;available(j)=false;
        if elapsed>1e-6
            measured=(found(i,1:2)-old(2:3))/elapsed;
            if norm(measured)<1.5,velocity=.5*velocity+.5*measured;end
        end
        tracks(j,:)=[old(1) found(i,:) velocity now hits];
    else
        tracks(end+1,:)=[nextId found(i,:) velocity now hits];nextId=nextId+1;available(end+1)=false; %#ok<AGROW>
    end
end
tracks=tracks(now-tracks(:,7)<=1,:);age=now-tracks(:,7);
dynamic=tracks(:,8)>=3 & vecnorm(tracks(:,5:6),2,2)>.04;
obs=[tracks(:,1) tracks(:,2:3)+tracks(:,5:6).*age tracks(:,4)+.1*age tracks(:,5:6).*dynamic dynamic];
end

function [map,known]=update_map(map,known,ranges,pose,obs,res,origin)
for i=1:3:numel(ranges)
    d=min(ranges(i),3.5);a=pose(3)-pi+(i-1)*2*pi/360;dir=[cos(a) sin(a)];
    endpoint=pose(1:2)+d*dir;last=floor((endpoint-origin)/res)+1;
    distances=(0:floor(d/(res*.5))-1)'*res*.5;
    cells=unique(floor((pose(1:2)+distances*dir-origin)/res)+1,'rows');
    cells=cells(~all(cells==last,2),:);
    valid=cells(:,1)>=1&cells(:,1)<=size(map,2)&cells(:,2)>=1&cells(:,2)<=size(map,1);
    cells=cells(valid,:);ids=sub2ind(size(map),cells(:,2),cells(:,1));map(ids)=max(-4,map(ids)-.35);known(ids)=true;
    moving=obs(obs(:,7)>0,:);isMoving=~isempty(moving)&&any(vecnorm(moving(:,2:3)-endpoint,2,2)<moving(:,4)+.2);
    if isfinite(ranges(i))&&~isMoving&&all(last>=1)&&last(1)<=size(map,2)&&last(2)<=size(map,1)
        map(last(2),last(1))=min(4,map(last(2),last(1))+.75);known(last(2),last(1))=true;
    end
end
end
function q=transform(points,p)
q=points*[cos(p(3)) sin(p(3));-sin(p(3)) cos(p(3))]+p(1:2);
end
function a=wrap(a), a=mod(a+pi,2*pi)-pi; end
