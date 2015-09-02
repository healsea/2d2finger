FeG = [eye(2);-ycenter];
FeN = inv(ror)*[-sqrt(1/2);-sqrt(1/2)]; % gravity orientation will change. It is always [-sqrt(1/2);-sqrt(1/2)] in world coordinate
Fe = FeG*FeN*Fenum;

xobjf = [xobjf;20;0;20;0];

Fg = -[mass;mass;rormass].*[ax;ay;beta1];  % inerital force
Fg = [ror*Fg(1:2);Fg(3)];

[xandf fval exitflag]= fmincon('maxf',xobjf,[],[],[],[],VLB,VUB,@(xandf) nonlinear1(xandf,U,R,Fe,Fg,ycenter))

% store resultnum
final(resultnum,:) = xandf;
resultnum = resultnum+1;
xobjf = xandf(1:4);

% renew velocity
vx = vx + 0.1*ax;
vy = vy + 0.1*ay;
omega = omega - 0.1*beta1;
% renew position
sx = vx*0.1+sx;  % 0.1 is time
sy = vy*0.1+sy;
theta = omega*0.1+theta;  % slip angle velocity
rorshi =[cos(theta) -sin(theta) sx;sin(theta) cos(theta) sy;0 0 1]; %update rorshi
ror = [cos(theta) -sin(theta) ;sin(theta) cos(theta)];

delete(objPos);
objPos = drawobj(yobj,rorshi); 
delete(line1);
delete(line2);
delete(finPos);
[finPos,line1,line2] = drawfin(ror,rorshi,R1,R2,xobjf);