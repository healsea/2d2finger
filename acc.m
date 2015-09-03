FeG = [eye(2);-ycenter];
FeN = inv(rorf)*[-sqrt(1/2);-sqrt(1/2)]; % gravity orientation will change. It is always [-sqrt(1/2);-sqrt(1/2)] in world coordinate
Fe = FeG*FeN*Fenum;

xobjf = [xobjf;20;0;20;0];

Fg = -[mass;mass;rormass].*[axf;ayf;beta1f];  % inerital force
Fg = [rorf*Fg(1:2);Fg(3)];

[xandf fval exitflag]= fmincon('maxf',xobjf,[],[],[],[],VLB,VUB,@(xandf) nonlinear1(xandf,U,R,Fe,Fg,ycenter));
% store resultnum
final(resultnum,:) = xandf;
MAXF(resultnum,:) = fval;
resultnum = resultnum+1;

xobjf = xandf(1:4);
outfunstore(outfunnum,:) = xobjf';
% renew velocity
vxf = vxf + 0.1*axf;
vyf = vyf + 0.1*ayf;
omegaf = omegaf + 0.1*beta1f;
% renew position
sxf = vxf*0.1+sxf;  % 0.1 is time
syf = vyf*0.1+syf;
thetaf = omegaf*0.1+thetaf;  % slip angle velocity
rorshif =[cos(thetaf) -sin(thetaf) sxf;sin(thetaf) cos(thetaf) syf;0 0 1]; %update rorshi
rorf = [cos(thetaf) -sin(thetaf) ;sin(thetaf) cos(thetaf)];

