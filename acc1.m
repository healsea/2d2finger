FeG = [eye(2);-ycenter];
FeN = inv(ror)*[-sqrt(1/2);-sqrt(1/2)]; % gravity orientation will change. It is always [-sqrt(1/2);-sqrt(1/2)] in world coordinate
Fe = FeG*FeN*Fenum;

Fg = -[mass;mass;rormass].*[ax;ay;beta1];  % inerital force
Fg = [ror*Fg(1:2);Fg(3)];

VLB1 = [5;-Inf;5;-Inf];  % low boundary
VUB1 = [Inf*ones(4,1)];  %high boundary

F = [20;0;20;0];
[F fval exitflag]= fmincon('maxf1',F,[],[],[],[],VLB1,VUB1,@(xandf) nonlinear2(F,U,R,Fe,Fg,ycenter,xobjf))

vxf = vxf + 0.1*axf;
vyf = vyf + 0.1*ayf;
omegaf = omegaf + 0.1*beta1f;
% renew position
sxf = vxf*0.1+sxf;  % 0.1 is time
syf = vyf*0.1+syf;
thetaf = omegaf*0.1+thetaf;  % slip angle velocity
rorshif =[cos(thetaf) -sin(thetaf) sxf;sin(thetaf) cos(thetaf) syf;0 0 1]; %update rorshi
rorf = [cos(thetaf) -sin(thetaf) ;sin(thetaf) cos(thetaf)];
