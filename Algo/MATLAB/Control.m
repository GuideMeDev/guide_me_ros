function Control(xplus,xminus)
sc=20;sizemx=5000/sc;sizemy=5200/sc;%k=12;d=[];for ki=-5:k;
dlen=round(2800/sc);dwi=round(250/sc);
%find the control matrix t2 based on the pixels in the SLAM of x within specific region  
x=xplus;xcurbe=xminus;
f=(x(:,1)>2)&x(:,1)<sizemx-1&(abs(x(:,2))<sizemy/2-1);px=round(x(f,1));
% defining image region for obstacles (above/below) showcase
py=round(x(f,2))+sizemy/2;t2=zeros(sizemy,sizemx);t2(((px-1).*size(t2,1)+py))=1;
%find the control matrix t2curbe based on the pixels in the SLAM of xcurbe within specific region  
f=(xcurbe(:,1)>1)&xcurbe(:,1)<sizemx&(abs(xcurbe(:,2))<sizemy/2);px=round(xcurbe(f,1));py=round(xcurbe(f,2))+sizemy/2;
t2curbe=zeros(sizemy,sizemx);t2curbe(((px-1).*size(t2curbe,1)+py))=1;
%create the control matrix - t2 above, t2curbe below
mbypass=fliplr(fliplr(t2curbe)');mbypass(:,:,3)=fliplr(fliplr(t2)');
%to change "below" level to orange color
mbypass(:,:,2)=fliplr(fliplr(t2curbe)')/2;
%to create in the control matrix the expected user trajectory - scan obstacles
mbypass(end-dlen:end,sizemy/2-dwi:sizemy/2+dwi,2)=1;
%to create in the control matrix the expected user bypass trajectory from
%left and right sides of the obstacle - 0.2 left, 0.1 right
mbypass(end-dlen:end,sizemy/2-2*dwi:sizemy/2-dwi,2)=0.2;mbypass(end-dlen:end,sizemy/2+dwi:sizemy/2+2*dwi,2)=0.1;
figure(2),subplot(2,2,[2,4]);imshow(double(mbypass))
delete(findall(gcf,'type','annotation'))
%search for overlap between obstacle(expected trajectory) and the user trajectory
obst=mbypass(:,:,1)+mbypass(:,:,3);scan=mbypass(:,:,2);t=(scan==1).*obst;
%if there is significant overlap with an obstacle
if sum(sum(t))>30
%search for overlap between the obstacle and the left and right bypass segments
s=[sum(sum(scan.*obst==0.2)),sum(sum(scan.*obst==0.1))];s=s==min(s);s=sum(s.*[1,2]);
%if the left segment has smaller overlap witht the obstacle, choose this bypass direction
if s==1
% title(['Crossing Angle = ',' Turn LefT']);locy = [0.35 0.35];locx = [0.35   0.3];a1=annotation('arrow',locx,locy);a1.Color='red';
xloc = [0.33 0.26]+0.4;yloc = [0.45 0.45]+0.2;annotation('textarrow',xloc,yloc,'String','Turn Left','Color','red','LineWidth',6,'FontSize',28);
%otherwise, choose the right bypass direction
else    
% title(['Crossing Angle = ',' Turn RighT']);locy = [0.4 0.4];lox = [0.3   0.35];a1=annotation('rightarrow',locx,locy);a1.Color='red';
xloc = [0.28 0.35]+0.4;yloc = [0.45 0.45]+0.2;annotation('textarrow',xloc,yloc,'String','Turn Right','Color','red','LineWidth',6,'FontSize',28);
end
% pause
end
title('Control')