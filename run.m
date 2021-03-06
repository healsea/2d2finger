
%%%%variables explanation%%%%
% xandf :Ã£â‚?he result of programming,which is the finger pos in obj coordinate
% xobjf :  4*1 matrix. only become 8*1 when to initialize xandf .the value is the same as xandf, it is used to initilize programming
% xworld : the finger pos in world coordinate
% yobj : object pos in obj coordinate, which is never change
% yworld : object pos in world coordinate
% final : store every results
% resultnum : how many results have been stored
% rorshi : rotate matrix with shift. to translate pos
% ror : rotate matrix without shift. to translate external force and normal vector
% objPos : handle of object figure
% finPos : handle of finger figure

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  initial
initial; %initial the value

% figure
figure;
axis equal
axis([-3.5,3,-2,4])  
hold on
pause;
% store optimal data from outfun so that we can draw it after optimalization
global outfunstore;
global outfunnum;
outfunnum = 1;
outfunstore = zeros(600,4);



%%%%%%%step 1 object pos in world coordinate%%%%%%
objPos = drawobj(yobj,rorshi);

%%%%%%%step 2 external force in obj coordinate%%%%%%
Fe = FeG*FeN*Fenum;


%%%%%%%step 3 make finger pos in obj coordinate meet the constraint%%%%%%

[finPos,line1,line2] = drawfin(ror,rorshi,R1,R2,xobjf);
pause(2);

%%%%%%%step 4 finger pos in obj coordinate%%%%%%

xobjf =[xobjf;20;0;20;0];
options = optimset('outputfcn',@outfun,'display','iter','Algorithm','active-set');%Ã¨Â®Â¾Ã§Â½Â®Ã¥Â¤â€“Ã©Æ’Â¨Ã¥â?Â½Ã¦â€¢Â°Ã£â‚¬?
[xandf,fval,exitflag]= fmincon('maxf',xobjf,[],[],[],[],VLB,VUB,@(xandf) nonlinear(xandf,U,R,Fe,ctact_mat,ctact_con),options);  % compute,the result stores in x(obj coordinate)

for i = 1:outfunnum-1
    delete(line1);
    delete(line2);
    delete(finPos);
    [finPos,line1,line2] = drawfin(ror,rorshi,R1,R2,outfunstore(i,:));   
    
    pause(0.5);
end

%store final result
final(resultnum,:) = xandf;
MAXF(resultnum,:) = fval;
resultnum = resultnum+1;


%%%%%%%step 5 finger pos in world coordinate%%%%%%
xobjf = xandf(1:4); % store next initial pos in obj coordinate



%%%%%%% step 7 compute force %%%%%%% 
% in order to make gif beautiful, in next part, I frist compute force. Then draw picture
sxf = sxf+vxf*30;
syf = syf+vyf*30;
thetaf = syf+omegaf*30;

for outfunnum = 1:200
    acc;
end
pause(2);


%%%%%%step 6 slip and find new obj coordinate%%%%%%


for k = 1:300 % regard 300 as reaction time
	sx = vx*0.1+sx;  % 0.1 is time
    sy = vy*0.1+sy;
    theta = omega*0.1+theta;  % slip angle velocity

    rorshi1 = rorshi;  % this is used to let finger is one step behind object
    rorshi =[cos(theta) -sin(theta) sx;sin(theta) cos(theta) sy;0 0 1]; %update rorshi
    ror = [cos(theta) -sin(theta) ;sin(theta) cos(theta)];
    
    delete(objPos);
    objPos = drawobj(yobj,rorshi);  

    % get the intersect of line and plane
    xobjf = intersect(yobj,rorshi,ror,rorshi1,xandf,xobjf,R);

    % judge weather the finger on the object exceed the maximum

    if(((xobjf<=(1.8*ones(4,1))) & ((xobjf>=[0.2;-0.1;0.2;0.2])))) % -0.1 to avoid -0 kind of thing
    else
    	if(xobjf(1)>1.8 || xobjf(1)<0.2)    %first finger
    		xobjf(1) = 1;
    	end
    	if(xobjf(3)>1.8 || xobjf(3)<0.2)    %second finger
    		xobjf(3) = 1;
    		xobjf(4) = 2-xobjf(3);
    	end
    end

    delete(line1);
    delete(line2);
    delete(finPos);
    [finPos,line1,line2] = drawfin(ror,rorshi,R1,R2,xobjf);

	pause(0.01);
end

for outfunnum =1:200
    vx = vx + 0.1*ax;
    vy = vy + 0.1*ay;
    omega = omega + 0.1*beta1;
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
    [finPos,line1,line2] = drawfin(ror,rorshi,R1,R2,outfunstore(outfunnum,:));
    pause(0.01);
end

pause(2)

%%%%%%%%% step 8 re-programmming at new world position
FeG = [eye(2);0 0]; 
FeN = inv(ror)*[-sqrt(1/2);-sqrt(1/2)]; % renew Fe as coordinate changed
Fe = FeG*FeN*Fenum;

xobjf =[xobjf;20;0;20;0];
outfunnum = 1;
options = optimset('outputfcn',@outfun,'display','iter','Algorithm','active-set');%Ã¨Â®Â¾Ã§Â½Â®Ã¥Â¤â€“Ã©Æ’Â¨Ã¥â?Â½Ã¦â€¢Â°Ã£â‚¬?
[xandf,fval,exitflag]= fmincon('maxf',xobjf,[],[],[],[],VLB,VUB,@(xandf) nonlinear(xandf,U,R,Fe,ctact_mat,ctact_con),options);  % compute,the result stores in x(obj coordinate)

for i = 1:outfunnum-1
    delete(line1);
    delete(line2);
    delete(finPos);
    [finPos,line1,line2] = drawfin(ror,rorshi,R1,R2,outfunstore(i,:));
    pause(0.5);
end

final(resultnum,:) = xandf;
MAXF(resultnum,:) = fval;
resultnum = resultnum+1;

xobjf = xandf(1:4); % store next initial pos in obj coordinate
pause(2);

%%%%%%%%% step 9 back  %%%%%%%%%%%%%%
%%% compute the force when back

for outfunnum = 1:200
    acc;
end
axf = 0;
ayf = 0;
beta1f = 0;
for outfunnum = 201:400
    acc;
end
axf = 0.0015;    
ayf = 0.0015;
beta1f = pi/2000;
for outfunnum = 401:600
    acc;
end

% draw the scene when back
ax = -0.0015;    
ay = -0.0015;
beta1 = -pi/2000;
for i =1:200
    vx = vx + 0.1*ax;
    vy = vy + 0.1*ay;
    omega = omega + 0.1*beta1;
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
    [finPos,line1,line2] = drawfin(ror,rorshi,R1,R2,outfunstore(i,:));
    pause(0.01);
end

ax = 0;
ay = 0;
beta1 = 0;
for i =201:400
    vx = vx + 0.1*ax;
    vy = vy + 0.1*ay;
    omega = omega + 0.1*beta1;
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
    [finPos,line1,line2] = drawfin(ror,rorshi,R1,R2,outfunstore(i,:));
    pause(0.01);
end


ax = 0.0015;    
ay = 0.0015;
beta1 = pi/2000;
for outfunnum =401:600
    vx = vx + 0.1*ax;
    vy = vy + 0.1*ay;
    omega = omega + 0.1*beta1;
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
    [finPos,line1,line2] = drawfin(ror,rorshi,R1,R2,outfunstore(i,:));
    pause(0.01);
end


pause(2);
%%%%%%%%% step 9 re-programmming when back to world position
FeG = [eye(2);0 0]; % the center of gravity don't change in object coordinate
FeN = inv(ror)*[-sqrt(1/2);-sqrt(1/2)]; % renew Fe as coordinate changed
Fe = FeG*FeN*Fenum;

xobjf =[xobjf;20;0;20;0];
outfunnum = 1;
options = optimset('outputfcn',@outfun,'display','iter','Algorithm','active-set');%Ã¨Â®Â¾Ã§Â½Â®Ã¥Â¤â€“Ã©Æ’Â¨Ã¥â?Â½Ã¦â€¢Â°Ã£â‚¬?
[xandf,fval,exitflag]= fmincon('maxf',xobjf,[],[],[],[],VLB,VUB,@(xandf) nonlinear(xandf,U,R,Fe,ctact_mat,ctact_con),options);  % compute,the result stores in x(obj coordinate)

for i = 1:outfunnum-1
    delete(line1);
    delete(line2);
    delete(finPos);
    [finPos line1 line2] = drawfin(ror,rorshi,R1,R2,outfunstore(i,:));     
    pause(0.5);
end

final(resultnum,:) = xandf;
MAXF(resultnum,:) = fval;

xobjf = xandf(1:4); % store next initial pos in obj coordinate

delete(line1);
delete(line2);
delete(finPos);
[finPos,line1,line2] = drawfin(ror,rorshi,R1,R2,xobjf);