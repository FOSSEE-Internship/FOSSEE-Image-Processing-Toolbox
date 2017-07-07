function [out]=dilate(input_image,actualkernel,anchor_x,anchor_y,iteration)
         input_image1=mattolist(input_image);
         a=opencv_dilate(input_image1,actualkernel,anchor_x,anchor_y,iteration);
         dimension=size(a)
         for i = 1:dimension
              out(:,:,i)=a(i);
         end
endfunction;
