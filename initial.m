clear
close all

% friction coefficient
U = 0.5;

% mass and moment of inertia
mass = 10;
rormass = 10*4/9;

% joint position
r0 = [5;0];

% store every results
final = zeros(1000,8);
resultnum = 1;
MAXF = zeros(1000,1);

%orientation matrix
rorshi = eye(3); %rotate combine with shift
ror = eye(2);

% initial position and velocity
sx = 0;
sy = 0;
theta = 0;
vx = 0.03;
vy = 0.03;
omega = pi/100;
ax = -0.0015;    %acceleration
ay = -0.0015;
beta1 = -pi/2000;    %angular acceleration

axf=ax; ayf=ay; beta1f=beta1; vxf=vx; vyf=vy; omegaf=omega; sxf=sx; syf=sy; thetaf=theta;
rorf = ror; rorshif=rorshi;
% boundary
VLB = [0.2;-0.1;0.2;0.2;5;-Inf;5;-Inf];  % low boundary
VUB = [1.8*ones(4,1);Inf*ones(4,1)];  %high boundary

%initial finger object position
xobjf =[1;0;1;1]; 

% initial objcet position
yobj = [0 0 2 0 0 2];   %object initial position,never change
ycenter = [2/3 2/3];    %object mass center position

% normal orientation of the object
R1 = [0 1;1 0]; % contact point on the bottom
R2 = [-sqrt(1/2) sqrt(1/2);-sqrt(1/2) -sqrt(1/2)];
R = blkdiag(R1,R2);

% force initial
FeG = [eye(2);0 0]; % the center of gravity don't change in object coordinate
FeN = inv(ror)*[-sqrt(1/2);-sqrt(1/2)]; % gravity orientation will change. It is always [-sqrt(1/2);-sqrt(1/2)] in world coordinate
Fenum = 30*sqrt(2); % the weight is 30N


