function [out]=pyramid(inputimage,direction,level)
// Computes the pyramid reduction or expansion of input_image
//
// Calling Sequence
// output_image=pyramid(inputimage,direction,level);
//
// Parameters
//
// inputimage : image matrix on which pyramid reduction or expansion has to be applied
// Direction  : An input string 'expand' for expand or 'reduce' for reduce the image matrix
// level      : It is specify number of times want to expand or reduce the dimension of an image matrix.
// output_image : Expanded or reduced form of image after pyramid reduction or expansion
//
// Description
// This function computes the pyramid reduction or expansion of input image.It uses  Gaussian pyramid to downsamples and upsamples the image pixels.In reduction it     reduces the size of image (if m*n matrix image) by m/pow(2,level) and n/pow(2,level).In expansion it expands the size of image (if m*n matrix image) by m*pow(2,level) and n*pow(2,level).Where pow(x,y)=x^y.
//
// Examples
// inputimage1 = imread('lena.jpeg');
// a=opencv_pyramid(inputimage1,'expand',2);
// imshow(a);
//
// Authors:
//
//Diwakar Bhardwaj



          inputimage1=mattolist(inputimage);
         a=opencv_pyramid(inputimage1,direction,level);
         dimension=size(a)
         for i = 1:dimension
              out(:,:,i)=a(i);
         end
     
endfunction;
