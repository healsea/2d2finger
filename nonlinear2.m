function [g,ceq] = nonlinear2(F,U,R,Fe,Fg,ycenter,xobjf)
%NONLINEAR describe the nonlinear constraint, specifically, the balance
%equation of force and moment


%the constraint that contact point must on the surface 

G =[eye(2) eye(2);VtoR(xobjf(1:2)-ycenter') VtoR(xobjf(3:4)-ycenter')]; 
ceq = G*R*F+Fe+Fg;  % equation constraint
ff = [F(1);abs(F(2));F(3);abs(F(4))];
g = [-1 1 0 0;0 0 -1 1]*ff;
end
