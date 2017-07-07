function [out]=filter(input_image,depth,kernel_matrix,anchor_x,anchor_y,delta)
         input_image1=mattolist(input_image);
         a=opencv_filter(input_image1,depth,kernel_matrix,anchor_x,anchor_y,delta);
         dimension=size(a)
         for i = 1:dimension
              out(:,:,i)=a(i);
         end
endfunction;
