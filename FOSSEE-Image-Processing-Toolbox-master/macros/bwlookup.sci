function [out]=bwlookup(image,lut)
         image1=mattolist(image);
         a=opencv_bwlookup(image1,lut);
         dimension=size(a)
         for i = 1:dimension
              out(:,:,i)=a(i);
         end
endfunction;
