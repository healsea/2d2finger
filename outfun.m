function stop = outfun(xandf,optimValues,state)
 stop = false;
     switch state

          case 'iter'
          disp([ 'the value of xandf']) ; 

          xobjf = xandf(1:4);
          
          global outfunstore;
          global outfunnum;
          outfunstore(outfunnum,:) = xobjf';
          outfunnum = outfunnum+1; 

     end
end