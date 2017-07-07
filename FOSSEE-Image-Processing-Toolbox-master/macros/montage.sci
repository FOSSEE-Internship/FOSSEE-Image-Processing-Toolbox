function [out]=montage(DataR,DataB,DataG)
         a=opencv_montage(DataR,DataB,DataG);
         dimension=size(a)
         for i = 1:dimension
              out(:,:,i)=a(i);
         end
     
endfunction;
