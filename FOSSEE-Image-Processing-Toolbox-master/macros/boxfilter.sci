function [out]=boxfilter(InputArraysrc, intddepth,size1x,size1y,pointx,pointy, bool_normalize)
         input_image1=mattolist(InputArraysrc);
         a=opencv_boxfilter(input_image1, intddepth,size1x,size1y,pointx,pointy, bool_normalize);
         dimension=size(a)
         for i = 1:dimension
              out(:,:,i)=a(i);
         end
     
endfunction;
