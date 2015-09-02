function objPos = drawobj(yobj,rorshi)
%DRAWOBJ is used to draw object
yworld = obj2world(yobj,rorshi);
ydraw1 = [yworld(1) yworld(3) yworld(5) yworld(1)];
ydraw2 = [yworld(2) yworld(4) yworld(6) yworld(2)];
objPos = fill(ydraw1,ydraw2,'b');
end