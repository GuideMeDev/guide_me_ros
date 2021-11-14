
%output: 
function [xplus,xminus] = SLAM(yawt_curr,minter_plus,minter_minus,xplus,xminus)
sc=20;kkx=6;kky=6;sizemx=5000/sc;sizemy=5200/sc;
c=conv2(minter_plus,ones(4,4)/16,'same');
c=(c>0.5);cminter_plus=c;%imshow(t2)%t2=mpc2*0;t2(k2:end-k2,k1:end-k2)=c;
%find convolved minter_minus (the filtered D-frame), by using low pass filter on minter_minus
c=conv2(minter_minus,ones(4,4)/16,'same');c=(c>0.5);cminter_minus=c;%imshow(t2)%t2=mpc2*0;t2(k2:end-k2,k1:end-k2)=c;
%step 1, transformation from pixels to coordinates, step 2, translation back in time of frame (i+1) to frame (i) 
[pyplus,pxplus]=find(cminter_plus>0);pyplus=pyplus-sizemy/2;px1plus=pxplus+yawt_curr(3);py1plus=pyplus+yawt_curr(2);
%same as above for pycurbe and pxcurbe, and for pyfloor and pxfloor
[pyminus,pxminus]=find(cminter_minus>0);pyminus=pyminus-sizemy/2;px1minus=pxminus+yawt_curr(3);py1minus=pyminus+yawt_curr(2);
%rotation back in time of frame (i+1) to frame (i) of plus and of minus
tetaz=-yawt_curr(1);Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];
tplus=(Rz*[px1plus';py1plus'])';tminus=(Rz*[pxminus';py1minus'])';
xplus=[[0,0];xplus;tplus];
xminus=[xminus;tminus];
%moving forward in time (rotation and translation) both SLAM of x and SLAM of xcurbe
tetaz=yawt_curr(1);Rz=[cos(tetaz),-sin(tetaz);sin(tetaz),cos(tetaz)];xplus=[(Rz*[xplus]')'];xminus=[(Rz*[xminus]')'];
xplus(:,1)=xplus(:,1)-yawt_curr(3);xplus(:,2)=xplus(:,2)-yawt_curr(2);xminus(:,1)=xminus(:,1)-yawt_curr(3);xminus(:,2)=xminus(:,2)-yawt_curr(2);
xplus=[xplus;[pxplus';pyplus']'];xminus=[xminus;[pxminus';pyminus']'];


%add the dframe (i+1) to the SLAM
%only for illustration in figures, find x1 (x, rotated by 90 degrees and scaled) and x1curbe (xcurbe, rotated by 90 degrees and scaled)
