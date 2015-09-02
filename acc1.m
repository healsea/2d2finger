FeG = [eye(2);-ycenter];
FeN = inv(ror)*[-sqrt(1/2);-sqrt(1/2)]; % gravity orientation will change. It is always [-sqrt(1/2);-sqrt(1/2)] in world coordinate
Fe = FeG*FeN*Fenum;

Fg = -[mass;mass;rormass].*[ax;ay;beta1];  % inerital force
Fg = [ror*Fg(1:2);Fg(3)];

VLB1 = [5;-Inf;5;-Inf];  % low boundary
VUB1 = [Inf*ones(4,1)];  %high boundary

F = [20;0;20;0];
[F fval exitflag]= fmincon('maxf1',F,[],[],[],[],VLB1,VUB1,@(xandf) nonlinear2(F,U,R,Fe,Fg,ycenter,xobjf))

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