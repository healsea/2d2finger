function [g,ceq] = nonlinear(xandf,U,R,Fe,Fg,ycenter)
%NONLINEAR describe the nonlinear constraint, specifically, the balance
%equation of force and moment
xx = xandf(1:4);  %coordinate
f = xandf(5:8);  %contact force

%the constraint that contact point must on the surface 

ctact_mat = [0 1 0 0 ;0 0 1 1]; % contact constraint 
ctact_con = [0;2];  
ceq1 = ctact_mat*xx - ctact_con;

G =[eye(2) eye(2);VtoR(xandf(1:2)-ycenter') VtoR(xandf(3:4)-ycenter')]; 
ceq2 = G*R*f+Fe+Fg;  % equation constraint
ceq = [ceq1;ceq2];

ff = [f(1);abs(f(2));f(3);abs(f(4))];
g = [-U 1 0 0;0 0 -U 1]*ff;

end
