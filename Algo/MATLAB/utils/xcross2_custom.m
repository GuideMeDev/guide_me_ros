function tx1=xcross2_custom(m1,m2,dyIMU,dxIMU,kkx,kky)
s=[];kx=-kkx:kkx;ky=-kky:kky;
tic;for j1=1:length(ky)
    for j2=1:length(kx)
        m2a=m2(size(m2,1)/2-size(m1,1)/2+1+ky(j1)-dyIMU:size(m2,1)/2+size(m1,1)/2+ky(j1)-dyIMU,size(m2,2)/2-size(m1,2)/2+1+kx(j2)-dxIMU:size(m2,2)/2+size(m1,2)/2+kx(j2)-dxIMU);
        s(j1,j2)=sum(sum(m1.*m2a));
    end
end 
% toc
% imagesc(s)
[fy,fx]=find((s)>max(max(s))*0.9);tx1=[dyIMU,dxIMU]-[ky(round(mean(fy))),kx(round(mean(fx)))];
