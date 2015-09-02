function xobjf = intersect(yobj,rorshi,ror,rorshi1,xandf,xobjf,R)
%INTERSECT gives the contact points of finger and object after slippery in obj coordinate

    %line direction
    xandfTemp = blkdiag(xandf(5:6),xandf(7:8));  % mutiply by R to get line direction
    linedirTemp = R*xandfTemp;
    linedir = [linedirTemp(1:2,1);linedirTemp(3:4,2)];  %4*1
    
    %line original point
    
    line0 = [reshape(xobjf,2,2);1 1];  % object initial world position
    line0 = rorshi1*line0;
    line0 = line0(1:2,:);
    line0 = reshape(line0,4,1);

    %line
    line = [reshape(line0,2,2)' reshape(linedir,2,2)'];  

    yobj = reshape(yobj,2,3);
    %edge direction
    edgedirTemp = [yobj(:,2)-yobj(:,1),yobj(:,3)-yobj(:,2)];   % edge direction in obj coordinate
    edgedir = ror*edgedirTemp;  % edge direction in world coordinate  
    edgedir = edgedir';

    %edge original point
    edge0Temp = [yobj(:,1),yobj(:,2);ones(1,2)];
    edge0 = rorshi*edge0Temp;
    edge0 = edge0(1:2,:)'; 

    edge = [edge0 edgedir];

    xworld= intersectLines(line,edge);
    xworld = xworld';


    xobjf = [reshape(xworld,2,2);1 1];  % object initial world position
    xobjf = inv(rorshi)*xobjf;
    xobjf = xobjf(1:2,:);
    xobjf = reshape(xobjf,4,1);
end
    








