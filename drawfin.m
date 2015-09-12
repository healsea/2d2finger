function [finPos line1 line2] = drawfin(ror,rorshi,R1,R2,xobjf)
%DRAWFIN is used to draw fingers and arms
r0 = [2;0]; % position of arm start point

n1 = -[cos(pi/4) -sin(pi/4);sin(pi/4) cos(pi/4)]*ror*R1(:,1);
n2 = -[cos(pi/4) -sin(pi/4);sin(pi/4) cos(pi/4)]*ror*R2(:,1);

xworld = obj2world(xobjf,rorshi);
xdraw = xworld +0.1*[n1;n2];    % find circle's center,0.1 is the radius

% draw arms 
xinter1 = inter(r0,xdraw(1:2));
xinter2 = inter(r0,xdraw(3:4));
xline1 = [r0 xinter1 xdraw(1:2)];
xline2 = [r0 xinter2 xdraw(3:4)];
line1 = plot(xline1(1,:),xline1(2,:),'y-','LineWidth',2);
line2 = plot(xline2(1,:),xline2(2,:),'y-','LineWidth',2);

% draw fingers
i = 0:pi/30:2*pi;
draw1 = 0.1*cos(i) + xdraw(1);
draw11 = 0.1*sin(i) + xdraw(2);
draw2 = 0.1*cos(i) + xdraw(3);
draw22 = 0.1*sin(i) + xdraw(4);
finPos = fill(draw1,draw11,'r',draw2,draw22,'r');
end