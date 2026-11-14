function videoPath=render_lidar_video(result,out)
% Frames rendered by MATLAB graphics and recorded with VideoWriter.
profiles=VideoWriter.getProfiles;profile='Motion JPEG AVI';ext='.avi';
if any(strcmp({profiles.Name},'MPEG-4')),profile='MPEG-4';ext='.mp4';end
videoPath=fullfile(out,['matlab_lidar_navigation' ext]);writer=VideoWriter(videoPath,profile);writer.FrameRate=10;writer.Quality=90;open(writer);
fig=figure('Color',[.04 .065 .10],'Position',[50 50 1280 720],'MenuBar','none','ToolBar','none','Resize','off');
cleanup=onCleanup(@() close(writer)); %#ok<NASGU>
for k=1:numel(result.frames)
 f=result.frames{k};clf(fig);bg=[.04 .065 .10];fg=[.83 .9 .96];
 annotation(fig,'textbox',[.045 .925 .9 .06],'String','LiDAR Navigation Lab | MATLAB','Color',fg,'FontSize',23,'FontWeight','bold','EdgeColor','none');
 annotation(fig,'textbox',[.047 .875 .9 .04],'String',sprintf('LIVE SIMULATION    %.1f s    |    %s    |    Replans: %d',f.t,upper(f.status),f.replans),'Color',[.25 .85 .8],'FontSize',12,'EdgeColor','none');
 ax=axes(fig,'Position',[.05 .19 .62 .65]);hold(ax,'on');set(ax,'Color',bg,'XColor',fg,'YColor',fg,'ZColor',fg,'GridColor',[.3 .4 .5]);grid(ax,'on');
 [x,y,z]=cylinder(1,32);
 for j=1:size(f.world,1)
  o=f.world(j,:);color=[.35 .45 .58];if o(6),color=[1 .53 .23];end
  surf(ax,o(1)+o(3)*x,o(2)+o(3)*y,.85*z,'FaceColor',color,'EdgeColor','none');
  fill3(ax,o(1)+o(3)*x(1,:),o(2)+o(3)*y(1,:),.85*ones(1,33),color);
  if o(6),quiver3(ax,o(1),o(2),1,2*o(4),2*o(5),0,0,'Color',color,'LineWidth',2);end
 end
 p=f.pose;surf(ax,p(1)+.5*x,p(2)+.5*y,.35*z,'FaceColor',[.1 .75 .8],'EdgeColor','none');
 fill3(ax,p(1)+.5*x(1,:),p(2)+.5*y(1,:),.35*ones(1,33),[.1 .75 .8]);
 quiver3(ax,p(1),p(2),.4,.7*cos(p(3)),.7*sin(p(3)),0,0,'Color','w','LineWidth',3);
 plot3(ax,f.trail(:,1),f.trail(:,2),.025*ones(size(f.trail,1),1),'Color',[.25 .85 .8],'LineWidth',2);
 if ~isempty(f.path),plot3(ax,f.path(:,1),f.path(:,2),.04*ones(size(f.path,1),1),'--','Color',[1 .8 .25],'LineWidth',2);end
 if ~isempty(f.points),plot3(ax,f.points(:,1),f.points(:,2),.08*ones(size(f.points,1),1),'.','Color',[.4 .9 1],'MarkerSize',5);end
 plot3(ax,9,0,.1,'p','MarkerSize',20,'MarkerFaceColor',[.4 .95 .5],'MarkerEdgeColor','w');
 axis(ax,[-1 10.5 -4 4 0 2]);view(ax,[-25 57]);xlabel(ax,'x (m)');ylabel(ax,'y (m)');title(ax,'Observer scene + estimated robot pose','Color',fg,'FontSize',12);
 ax2=axes(fig,'Position',[.73 .49 .23 .32]);prob=1./(1+exp(-f.map));prob(~f.observed)=.5;
 imagesc(ax2,f.origin(1)+(0:100)*f.resolution,f.origin(2)+(0:80)*f.resolution,prob,[0 1]);axis(ax2,'xy');axis(ax2,'equal');axis(ax2,[-1 10.5 -4 4]);colormap(ax2,flipud(gray));hold(ax2,'on');plot(ax2,f.trail(:,1),f.trail(:,2),'Color',[0 .7 .7]);plot(ax2,9,0,'gp','MarkerFaceColor','g');set(ax2,'XColor',fg,'YColor',fg);title(ax2,'LiDAR-built occupancy map','Color',fg);
 ax3=axes(fig,'Position',[.73 .22 .23 .17]);bar(ax3,f.basis,'FaceColor',[.2 .8 .8]);ylim(ax3,[0 1]);set(ax3,'Color',bg,'XColor',fg,'YColor',fg,'XTick',1:7,'XTickLabel',{'NB','NM','NS','ZE','PS','PM','PB'});title(ax3,'Actual fuzzy rule activations','Color',fg);
 annotation(fig,'textbox',[.05 .04 .92 .10],'String',sprintf('v = %.2f m/s    |    turn = %.2f rad/s    |    local pose corrections: %d\nGray: stationary   Orange: moving   Yellow: replanned route   Cyan: LiDAR / traveled path\nSensor-only navigation | Known target, unknown obstacles | Local SLAM (no loop closure) | No payload / MPC',f.command(1),f.command(2),f.corrections),'Color',fg,'FontSize',10,'EdgeColor','none');
 drawnow;frame=getframe(fig);writeVideo(writer,frame);
 if k==1,imwrite(frame.cdata,fullfile(out,'preview.png'));end
 if mod(k,50)==0,fprintf('Rendered %d / %d frames\n',k,numel(result.frames));end
end
for j=1:20,writeVideo(writer,frame);end
imwrite(frame.cdata,fullfile(out,'final_frame.png'));close(fig);
end
