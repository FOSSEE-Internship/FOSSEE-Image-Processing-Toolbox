function [out]=rectangle(input_image , x_coordinate_of_point1 , y_coordinate_point1, x_coordinate_of_point2 , y_coordinate_point2 , r_value,g_value,b_value,thickness,linetype,shift)
          input_image1=mattolist(input_image);
         a=opencv_rectangle(input_image , x_coordinate_of_point1 , y_coordinate_point1, x_coordinate_of_point2 , y_coordinate_point2 , r_value,g_value,b_value,thickness,linetype,shift);
         dimension=size(a)
         for i = 1:dimension
              out(:,:,i)=a(i);
        end
     
endfunction;
