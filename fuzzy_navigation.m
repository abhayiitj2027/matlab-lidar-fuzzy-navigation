function [command,c,status,basis,stats]=fuzzy_navigation(p,path,obs,measured,c,goal)
% Gaussian singleton fuzzy steering, bounded adaptation, VO and DWA filter.
dt=.05; stats=struct('voRejected',0);command=[0 0];status='Track';
if isempty(path),path=[p(1:2);goal];end
clearance=inf;if ~isempty(obs),clearance=min(vecnorm(obs(:,2:3)-p(1:2),2,2)-obs(:,4)-.6);end
[~,idx]=min(vecnorm(path-p(1:2),2,2));target=path(end,:);look=.6;if clearance<.5,look=.25;end
for j=idx:size(path,1),if norm(path(j,:)-p(1:2))>=look,target=path(j,:);break;end,end
moving=obs(obs(:,7)>0,:);evade=false;reverse=false;
for j=1:size(moving,1)
 if vo(p,0,moving(j,:),4,.3)
  away=p(1:2)-moving(j,2:3);target=p(1:2)+1.5*away/max(norm(away),1e-6);evade=true;break
 end
end
e=wrap(atan2(target(2)-p(2),target(1)-p(1))-p(3));
if evade&&abs(e)>pi/2,reverse=true;e=wrap(e+pi);end
basis=exp(-.5*((e-[-pi -1.2 -.4 0 .4 1.2 pi])./[.65 .45 .25 .2 .25 .45 .65]).^2);basis=basis/sum(basis);
mu=[max(0,min(1,(1.5-clearance)/.7)) max(0,min((clearance-1)/.8,(2.8-clearance))) max(0,min(1,(clearance-2.2)/1.3))];
if ~isfinite(clearance),mu=[0 0 1];end
mu=mu/max(sum(mu),eps);factor=mu*[.45;.8;1];omega=mu*[.6;1;1.25];
nominal=[.65*factor*(basis*[0;.1;.7;1;.7;.1;0])*min(1,norm(goal-p(1:2))/.6) omega*(basis*[-1;-.9;-.5;0;.5;.9;1])];
if evade,nominal(1)=(1-2*reverse)*min(.3,max(.12,abs(nominal(1))));end
if ~isempty(c.basis),residual=max(-.3,min(.3,c.lastW-measured(2)));c.weights=max(-.12,min(.12,c.weights+dt*(.2*c.basis*residual-.05*c.weights)));end
nominal(2)=nominal(2)+dot(c.weights,basis);
lo=max([-.3 -1.25],measured-[.6 1.5]*dt);hi=min([.65 1.25],measured+[.6 1.5]*dt);
vs=unique([linspace(lo(1),hi(1),5) max(lo(1),min(hi(1),nominal(1)))]);ws=unique([linspace(lo(2),hi(2),9) max(lo(2),min(hi(2),nominal(2)))]);best=inf;
for v=vs
 for w=ws
  if any_vo(p,v,moving),stats.voRejected=stats.voRejected+1;continue;end
  [ok,q,gap]=rollout(p,v,w,obs,false);if ~ok,continue;end
  [ok,~,~]=rollout(p,v,w,obs,true);if ~ok,continue;end
  error=abs(wrap(atan2(target(2)-q(2),target(1)-q(1))-q(3)+reverse*pi));
  cost=2*(v-nominal(1))^2+1.5*(w-nominal(2))^2+.35*norm(q(1:2)-target)+.15*error-.05*min(gap,1);
  if cost<best,best=cost;command=[v w];end
 end
end
if isinf(best),status='Yield';elseif evade,status='Evade';elseif abs(command(1)-nominal(1))>.08||abs(command(2)-nominal(2))>.15,status='Avoid';end
if norm(goal-p(1:2))<.2,command=[0 0];status='Goal';end
c.basis=basis;c.lastW=command(2);
end
function yes=any_vo(p,v,obs)
yes=false;for j=1:size(obs,1),if vo(p,v,obs(j,:),2,.12),yes=true;return;end,end
end
function yes=vo(p,v,o,horizon,margin)
a=o(2:3)-p(1:2);u=v*[cos(p(3)) sin(p(3))]-o(5:6);t=max(0,min(horizon,dot(a,u)/max(dot(u,u),1e-12)));yes=norm(a-u*t)<.6+o(4)+margin;
end
function [ok,p,gap]=rollout(p,v,w,obs,brake)
ok=true;gap=inf;
for k=1:20
 if brake&&k>1,v=sign(v)*max(0,abs(v)-.06);w=sign(w)*max(0,abs(w)-.15);end
 p=p+[v*cos(p(3))*.1 v*sin(p(3))*.1 w*.1];
 if ~isempty(obs)
  g=min(vecnorm(obs(:,2:3)+obs(:,5:6)*k*.1-p(1:2),2,2)-obs(:,4)-.6);gap=min(gap,g);
  if g<.08,ok=false;return;end
 end
end
end
function a=wrap(a),a=mod(a+pi,2*pi)-pi;end
