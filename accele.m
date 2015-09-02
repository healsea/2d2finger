yworld = obj2world(yobj,rorshi);
ycenterworld = obj2world(ycenter,rorshi);
xworld = obj2world(xobjf,rorshi);

% ask for force
FeG = [eye(2);VtoR(yworld(1:2)-ycenterworld)]; % the center of gravity don't change in object coordinate
FeN = [-sqrt(1/2);-sqrt(1/2)]; % we are doing this in world coordinate
Fe = FeG*FeN*Fenum;

a = 0.0003;    %acceleration
beta1 = pi/10000;    %angular acceleration

Rworld = blkdiag(ror,ror)*R;
Gworld = [eye(2) eye(2);VtoR(xobjf(1:2)-ycenterworld) VtoR(xobjf(1:2)-ycenterworld)];
VLBworld = [5;-Inf;5;-Inf];
VUBworld = Inf*ones(4,1); 
aim = [1;0;1;0]
Aeq = Gworld*Rworld;
Beq = [mass;0;rormass].*[a 0 beta1] - Fe;
A = [-U 1 0 0;0 0 -U 1;-U -1 0 0;0 0 -U -1];
B = [0;0;0;0];
F = linprog(aim,A,B,Aeq,Beq,,VLBworld,VUBworld);

% renew velocity
dis = dis - a;
omega = omega - beta1;
% renew position
rorshi =[cos(omega) -sin(omega) -dis;sin(omega) cos(omega) 0;0 0 1]+rorshi; %update rorshi
ror = [cos(omega) -sin(omega) ;sin(omega) cos(omega)]+ror;