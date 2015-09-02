function R1 = VtoR(r)
%VTOR is used to translate a vector to a rotate matrix, which is
% antisymmetric. 

R1 = zeros(1,2);
R1(1) = -r(2);
R1(2) = r(1);

end