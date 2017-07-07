function [out]=ellipse(input_image , x_coordinate_of_centre , y_coordinate_centre , first_axis, second_axis, angle, startAngle, endAngle, r_value,g_value,b_value,thickness,linetype,shift)
input_image1=mattolist(input_image);
         a=opencv_ellipse(input_image1 , x_coordinate_of_centre , y_coordinate_centre , first_axis, second_axis, angle, startAngle, endAngle, r_value,g_value,b_value,thickness,linetype,shift);
         dimension=size(a)
         for i = 1:dimension
              out(:,:,i)=a(i);
         end
     
endfunction;
