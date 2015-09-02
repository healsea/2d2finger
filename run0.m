clear

%%%%variables explanation%%%%
% xandf :�?he result of programming,which is the finger pos in obj coordinate
% xobjf :  4*1 matrix. only become 8*1 when to initialize xandf .the value is the same as xandf, it is used to initilize programming
% xworld : the finger pos in world coordinate
% yobj : object pos in obj coordinate, which is never change
% yworld : object pos in world coordinate
% final : store every results
% resultnum : how many results have been stored
% rorshi : rotate matrix with shift. to translate pos
% ror : rotate matrix without shift. to translate external force
% objPos : handle of object figure
% finPos : handle of finger figure

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  initial
% figure
figure
axis([-2,4,-2,4])  
axis equal
hold on

% friction coefficient
global U;
U = 0.5;

% store every results
final = zeros(100,8);
resultnum = 1;

%orientation matrix
global rorshi;
global ror;
rorshi = eye(3); %rotate combine with shift
ror = eye(2);

% boundary
VLB = [0.2;-0.1;0.2;0.2;5;-Inf;5;-Inf];  % low boundary
VUB = [1.8*ones(4,1);Inf*ones(4,1)];  %high boundary
xobjf =[1;0;1;1]; %initial finger object position

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% object
% This m file describe the position and orientation of the object to be grasped
% ALL data is measured in the object coordinate, not world coordinate

yobj = [0 0 2 0 0 2];   %object initial position,never change

% normal orientation of the object
global R1 R2;
R1 = [0 1;1 0]; % contact point on the bottom
R2 = [-sqrt(1/2) sqrt(1/2);-sqrt(1/2) -sqrt(1/2)];
global R;
R = blkdiag(R1,R2);


%%%%%%%step 1 object pos in world coordinate%%%%%%
yworld = [reshape(yobj,2,3);1 1 1];  % object initial world position
yworld = rorshi*yworld;
yworld = yworld(1:2,:);
yworld = reshape(yworld,6,1);
ydraw1 = [yworld(1) yworld(3) yworld(5) yworld(1)];
ydraw2 = [yworld(2) yworld(4) yworld(6) yworld(2)];
objPos = fill(ydraw1,ydraw2,'b');

%%%%%%%step 2 external force in obj coordinate%%%%%%

FeG = [eye(2);0 0]; % the center of gravity don't change in object coordinate
FeN = inv(ror)*[-sqrt(1/2);-sqrt(1/2)]; % gravity orientation will change. It is always [-sqrt(1/2);-sqrt(1/2)] in world coordinate
Fenum = 30*sqrt(2); % the weight is 10N
global Fe;
Fe = FeG*FeN*Fenum;

%%%%%%%step 3 make finger pos in obj coordinate meet the constraint%%%%%%
xobjf(2) = 0;
xobjf(4) = 2 - xobjf(3);

n1 = -ror*R1(:,1);
n2 = -ror*R2(:,1);

xworld = [reshape(xobjf,2,2);1 1];  % object initial world position
xworld = rorshi*xworld;
xworld = xworld(1:2,:);
xworld = reshape(xworld,4,1);
xdraw = xworld +0.1*[n1;n2];    % find circle's center,0.1 is the radius
i = 0:pi/30:2*pi;
draw1 = 0.1*cos(i) + xdraw(1);
draw11 = 0.1*sin(i) + xdraw(2);
draw2 = 0.1*cos(i) + xdraw(3);
draw22 = 0.1*sin(i) + xdraw(4);

finPos = fill(draw1,draw11,'r',draw2,draw22,'r');
pause(1)
delete(finPos);

global fmat j;
j = 2; % make the index in outfun.m correct
fmat = moviein(100); % create movie matrix
fmat(:,1) = getframe;

%%%%%%%step 4 finger pos in obj coordinate%%%%%%
xobjf =[xobjf;20;0;20;0];
options = optimset('outputfcn',@outfun,'display','iter','Algorithm','active-set');%设置外部函数�?
[xandf fval exitflag]= fmincon('maxf',xobjf,[],[],[],[],VLB,VUB,'nonlinear',options)  % compute,the result stores in x(obj coordinate)
final(resultnum,:) = xandf;
resultnum = resultnum+1;

%%%%%%%step 5 finger pos in world coordinate%%%%%%
xobjf = xandf(1:4); % store next initial pos in obj coordinate

n1 = -ror*R1(:,1);
n2 = -ror*R2(:,1);

xworld = [reshape(xobjf,2,2);1 1];  % object initial world position
xworld = rorshi*xworld;
xworld = xworld(1:2,:);
xworld = reshape(xworld,4,1);
xdraw = xworld +0.1*[n1;n2];    % find circle's center,0.1 is the radius
i = 0:pi/30:2*pi;
draw1 = 0.1*cos(i) + xdraw(1);
draw11 = 0.1*sin(i) + xdraw(2);
draw2 = 0.1*cos(i) + xdraw(3);
draw22 = 0.1*sin(i) + xdraw(4);

finPos = fill(draw1,draw11,'r',draw2,draw22,'r');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% slip
%%%%%%step 6 slip and find new obj coordinate%%%%%%

for k = 1:300  % regard 100 as reaction time反应时间
	dis =  0.003;  % slip velocity
    omega = pi/1000;  % slip angle velocity

    rorshi =[cos(omega) -sin(omega) -dis;sin(omega) cos(omega) 0;0 0 1]*rorshi; %update rorshi
    ror = [cos(omega) -sin(omega) ;sin(omega) cos(omega)]*ror;
    
    yworld = [reshape(yobj,2,3);1 1 1];  % object initial world position
    yworld = rorshi*yworld;
    yworld = yworld(1:2,:);
    yworld = reshape(yworld,6,1);
    ydraw1 = [yworld(1) yworld(3) yworld(5) yworld(1)];
    ydraw2 = [yworld(2) yworld(4) yworld(6) yworld(2)];
    delete(objPos);
    objPos = fill(ydraw1,ydraw2,'b');
    
	
    dir1 = xandf(1)*R1(:,1)+xandf(2)*R1(:,2); % direction of motion/activateForce of the first contact point
    dir2 = xandf(3)*R2(:,1)+xandf(4)*R2(:,2); % direction of motion/activateForce of the second contact point

    inter1 = [yworld(4)-yworld(2),yworld(1)-yworld(3);dir1(2),-dir1(1)]\[yworld(4)*yworld(1)-yworld(2)*yworld(3);dir1(2)*xworld(1)-dir1(1)*xworld(2)];
    inter2 = [yworld(4)-yworld(6),yworld(5)-yworld(3);dir2(2),-dir2(1)]\[yworld(4)*yworld(5)-yworld(6)*yworld(3);dir2(2)*xworld(3)-dir2(1)*xworld(4)];

    xworld(1) = inter1(1);
    xworld(2) = inter1(2);
    xworld(3) = inter2(1);
    xworld(4) = inter2(2);

    % judge weather the finger on the object exceed the maximum
    xobjf = [reshape(xworld,2,2);1 1];  % object initial world position
    xobjf = inv(rorshi)*xobjf;
    xobjf = xobjf(1:2,:);
    xobjf = reshape(xobjf,4,1);
    if(((xobjf<=(1.8*ones(4,1))) & ((xobjf>=[0.2;-0.1;0.2;0.2])))) % -0.1 to avoid -0 kind of thing
    else
    	if(xobjf(1)>1.8 || xobjf(1)<0.2)    %first finger
    		xobjf(1) = 1;
    	end
    	if(xobjf(3)>1.8 || xobjf(3)<0.2)    %second finger
    		xobjf(3) = 1;
    		xobjf(4) = 2-xobjf(3);
    	end
    	xworld = [reshape(xobjf,2,2);1 1];  % renew changed fingers position
        xworld = rorshi*xworld;
        xworld = xworld(1:2,:);
        xworld = reshape(xworld,4,1);
    end

    n1 = -ror*R1(:,1);
    n2 = -ror*R2(:,1);

	xdraw = xworld +0.1*[n1;n2];    % find circle's center,0.1 is the radius
	i = 0:pi/30:2*pi;
	draw1 = 0.1*cos(i) + xdraw(1);
	draw11 = 0.1*sin(i) + xdraw(2);
	draw2 = 0.1*cos(i) + xdraw(3);
	draw22 = 0.1*sin(i) + xdraw(4);

    delete(finPos);
	finPos = fill(draw1,draw11,'r',draw2,draw22,'r');

	if  (mod(k,15) == 0)
		fmat(:,j) = getframe;
        j = j+1;
    end
	pause(0.02);
end


%%%%%%%%% step 7 re-programmming at new world position
FeN = inv(ror)*[-sqrt(1/2);-sqrt(1/2)]; % renew Fe as coordinate changed
Fe = FeG*FeN*Fenum;

xobjf =[xobjf;20;0;20;0];
options = optimset('outputfcn',@outfun,'display','iter','Algorithm','active-set');%设置外部函数�?
[xandf fval exitflag]= fmincon('maxf',xobjf,[],[],[],[],VLB,VUB,'nonlinear',options)  % compute,the result stores in x(obj coordinate)
final(resultnum,:) = xandf;
resultnum = resultnum+1;

xobjf = xandf(1:4); % store next initial pos in obj coordinate

n1 = -ror*R1(:,1);
n2 = -ror*R2(:,1);

xworld = [reshape(xobjf,2,2);1 1];  % object initial world position
xworld = rorshi*xworld;
xworld = xworld(1:2,:);
xworld = reshape(xworld,4,1);
xdraw = xworld +0.1*[n1;n2];    % find circle's center,0.1 is the radius
i = 0:pi/30:2*pi;
draw1 = 0.1*cos(i) + xdraw(1);
draw11 = 0.1*sin(i) + xdraw(2);
draw2 = 0.1*cos(i) + xdraw(3);
draw22 = 0.1*sin(i) + xdraw(4);
delete(finPos);
finPos = fill(draw1,draw11,'r',draw2,draw22,'r');

pause(2);

%%%%%%%%% step 8 back  %%%%%%%%%%%%%%


for k = 1:300  % regard 100 as reaction time反应时间
	dis =  -0.003;  % slip velocity
    omega = -pi/1000;  % slip angle velocity

    rorshi =[cos(omega) -sin(omega) -dis;sin(omega) cos(omega) 0;0 0 1]*rorshi; %update rorshi
    ror = [cos(omega) -sin(omega) ;sin(omega) cos(omega)]*ror;
    
    yworld = [reshape(yobj,2,3);1 1 1];  % object initial world position
    yworld = rorshi*yworld;
    yworld = yworld(1:2,:);
    yworld = reshape(yworld,6,1);
    ydraw1 = [yworld(1) yworld(3) yworld(5) yworld(1)];
    ydraw2 = [yworld(2) yworld(4) yworld(6) yworld(2)];
    delete(objPos);
    objPos = fill(ydraw1,ydraw2,'b');

	n1 = -ror*R1(:,1);
	n2 = -ror*R2(:,1);

	xworld = [reshape(xobjf,2,2);1 1];  % object initial world position
	xworld = rorshi*xworld;
	xworld = xworld(1:2,:);
	xworld = reshape(xworld,4,1);
	xdraw = xworld +0.1*[n1;n2];    % find circle's center,0.1 is the radius
	i = 0:pi/30:2*pi;
	draw1 = 0.1*cos(i) + xdraw(1);
	draw11 = 0.1*sin(i) + xdraw(2);
	draw2 = 0.1*cos(i) + xdraw(3);
	draw22 = 0.1*sin(i) + xdraw(4);
	delete(finPos);
	finPos = fill(draw1,draw11,'r',draw2,draw22,'r');

    if (mod(k,15) == 0)
		fmat(:,j) = getframe;
        j = j+1;
    end
    pause(0.02);
end

%%%%%%%%% step 7 re-programmming when back to world position
FeN = inv(ror)*[-sqrt(1/2);-sqrt(1/2)]; % renew Fe as coordinate changed
Fe = FeG*FeN*Fenum;






xobjf =[xobjf;20;0;20;0];
options = optimset('outputfcn',@outfun,'display','iter','Algorithm','active-set');%设置外部函数�?
[xandf fval exitflag]= fmincon('maxf',xobjf,[],[],[],[],VLB,VUB,'nonlinear',options)  % compute,the result stores in x(obj coordinate)
final(resultnum,:) = xandf;
resultnum = resultnum+1;

xobjf = xandf(1:4); % store next initial pos in obj coordinate

n1 = -ror*R1(:,1);
n2 = -ror*R2(:,1);

xworld = [reshape(xobjf,2,2);1 1];  % object initial world position
xworld = rorshi*xworld;
xworld = xworld(1:2,:);
xworld = reshape(xworld,4,1);
xdraw = xworld +0.1*[n1;n2];    % find circle's center,0.1 is the radius
i = 0:pi/30:2*pi;
draw1 = 0.1*cos(i) + xdraw(1);
draw11 = 0.1*sin(i) + xdraw(2);
draw2 = 0.1*cos(i) + xdraw(3);
draw22 = 0.1*sin(i) + xdraw(4);
delete(finPos);
finPos = fill(draw1,draw11,'r',draw2,draw22,'r');


fmat(:,j) = getframe;
j = j+1;