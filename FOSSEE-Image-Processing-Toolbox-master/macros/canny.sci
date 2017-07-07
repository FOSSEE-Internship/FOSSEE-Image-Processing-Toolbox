function [out]=canny(input_image ,aperture, threshold1, threshold2, gradient)
         input_image1=mattolist(input_image);
         a=opencv_canny(input_image1 , aperture, threshold1, threshold2, gradient);
         dimension=size(a)
         for i = 1:dimension
              out(:,:,i)=a(i);
         end
     
endfunction;
