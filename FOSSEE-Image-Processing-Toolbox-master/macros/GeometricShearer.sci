function [out] = GeometricShearer(inputimage,direction,value)
// Shear the image
//
// Calling Sequence 
// output_image = GeometricShearer(inputimage,direction,value);
//
// Parameters
//
// inputimage : image matrix on which geometric shear has to be applied
// direction : An input string that like. 'Horizontal' to shear the image in horizontal direction or 'Vertical' to shear the image in vertical direction
// value : A scalar value i.e amount to want shift the pixel of image
// output_image : sheared image
//
// Description
// This function returns the sheared image.It shears the image by shifting the rows or columns of image.
//
// Examples
// a = imread('lena.jpeg');
// b = GeometricShearer(a,'Horzontal',5);
// imshow(b);
//
// Authors:
//
//Diwakar Bhardwaj

          inputimage1=mattolist(inputimage);
          a = raw_GeometricShearer(inputimage1,direction,value);
          dimension=size(a)
          for i = 1:dimension
              out(:,:,i)=a(i);
          end

endfunction;
