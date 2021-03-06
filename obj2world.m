function world = obj2world(obj,rorshi)
	%OBJ2WORLD transform coordinate in obj to world. require n*1 to n*1
    num = length(obj);
	world = [reshape(obj,2,num/2);ones(1,num/2)];  % object initial world position
    world = [cos(pi/4) -sin(pi/4) 0 ;sin(pi/4) cos(pi/4) 0;0 0 1]*rorshi*world;
    world = world(1:2,:);
	world = reshape(world,num,1);
end