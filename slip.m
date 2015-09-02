for i = 1:300  % regard 100 as reaction time反应时间
	dis = 0.003;
	yobj = yobj - [dis 0 dis 0 dis 0];
	delete(objPos);
	objPos = plot([yobj(1) yobj(3)],[yobj(2) yobj(4)],'-',[yobj(1) yobj(5)],[yobj(2) yobj(6)],'-',[yobj(5) yobj(3)],[yobj(6) yobj(4)],'-');  % draw the initial position of the thing
    la = 120*[-sqrt(1/2) -sqrt(1/2)]+60*[sqrt(1/2) -sqrt(1/2)]; % direction of motion of the second contact point
    
    inter = [1,1;la(2),-la(1)]\[yobj(3);xobjf(3)*la(2) - xobjf(4)*la(1)];
    xobjf(3) = inter(1);
    xobjf(4) = inter(2);
    delete(finPos);
	finPos = plot(xobjf(1),xobjf(2),'ro',xobjf(3),xobjf(4),'ro');
	pause(0.02);
end







