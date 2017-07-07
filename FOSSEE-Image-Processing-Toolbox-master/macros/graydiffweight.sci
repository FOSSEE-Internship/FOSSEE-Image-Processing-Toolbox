function [out]=graydiffweight(image,refgrayval)
           image1=mattolist(image);
         a=opencv_graydiffweight(image1,refgrayval);
         dimension=size(a)
         for i = 1:dimension
              out(:,:,i)=a(i);
         end
     
endfunction;
