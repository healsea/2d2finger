function rinter = inter(r0,r)
% INTER calculated the inter-joint position when hand's manipulator arrives at a given point.
L = 3; % length of one leg.

if(norm(r-r0) > 6)
	error('so far for hands to reach');
end

rr = 1/2*(r+r0);
unitvec = (r-r0)/norm(r-r0);
h = sqrt(L^2-(norm(rr-r0)^2));  % Pythagorean theorem
rinter = rr+h*[-unitvec(2);unitvec(1)];
end

