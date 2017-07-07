function [out]=IFFT(inputimage)
// Computes the inverse descrete fourier transform of image
//
// Calling Sequence
// output_image = IFFT(inputimage);
//
// Parameters
//
// inputimage : Grayscale image
// out_image  : IFFT of input image
//
// Description
// This function computes the inverse descrete fourier transform of input image.The image should be grayscale.
//
// Examples
// a = imread('cameraman.tif');
// b = IFFT(a);
// imshow(b)
//
// Authors:
//
//Diwakar Bhardwaj


          inputimage1=mattolist(inputimage);
          a=opencv_IFFT(inputimage1);
          dimension=size(a)
          for i = 1:dimension
              out(:,:,i)=a(i);
          end
endfunction;
