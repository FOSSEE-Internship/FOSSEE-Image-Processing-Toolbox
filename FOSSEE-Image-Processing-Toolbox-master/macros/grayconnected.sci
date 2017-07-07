function [out]=grayconnected(input1,input2,input3,varargin)
         inputn=mattolist(input1);
         [lhs rhs] = argn(0)
         if rhs>4 then
         error(msprintf("Too many input arguments"))
         elseif rhs==4 then
            a=opencv_grayconnected(inputn,input2,input3,varargin(1));
            dimension=size(a)
            for i = 1:dimension
              out(:,:,i)=a(i);
            end
         else
            a=opencv_grayconnected(inputn,input2,input3);
            dimension=size(a)
            for i = 1:dimension
              out(:,:,i)=a(i);
            end
         end,

endfunction;
         
