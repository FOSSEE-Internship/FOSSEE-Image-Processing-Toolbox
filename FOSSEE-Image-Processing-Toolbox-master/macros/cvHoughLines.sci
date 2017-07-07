function [points] = cvHoughLines(image,theta,rho)
// Cartesian co-ordinates of points defined by rho and theta pairs.
//
// Calling Sequence
// points = cvHoughLines(image, theta, rho)
//
// Parameters
// image : input image matrix
// theta : vector of values of theta that represent the input line
// rho : vector of values of theta that represent the input line
// points : It is an MX4 matrix containing the points of intersection of the line(represented by rho and theta) with the image boundary.  
//
// Description
//
// The function returns the points of intersection of the line defined by rho and theta pairs with the image boundary. Image boundary refers to the left and right vertical boundary and top and bottom horizontal boundary.
//
// Examples
// //Load an image
// I = imread('lena.jpeg');
// // Calculate the points of intersection
// points = cvHoughLines(I,[pi/3 pi/6],[5 9]);
// 
// Authors
// Asmita Bhar
//

	img = mattolist(image);
	points = opencv_HoughLines(img,theta,rho);
endfunction
