function [out]=histeq(image)
         image1=mattolist(image);
         a=opencv_histeq(image1);
         dimension=size(a)
         for i = 1:dimension
              out(:,:,i)=a(i);
         end
     
endfunction;
