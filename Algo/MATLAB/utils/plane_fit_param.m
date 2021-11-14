function s=plane_fit_param(x2)
fin=find(abs(x2(:,2))<0.5&abs(x2(:,1)-3)<0.5&abs(x2(:,3))<0.08);
fout=find(abs(x2(:,2))>0.5&abs(x2(:,2))<1.5&abs(x2(:,1)-3)>0.5&abs(x2(:,3))<0.08);
s=std(x2(fin,3))/std(x2(fout,3));

