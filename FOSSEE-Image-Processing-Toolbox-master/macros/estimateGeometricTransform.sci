function [output] = estimateGeometricTransform(img1,img2)
	I1 = mattolist(img1);
	I2 = mattolist(img2);	
	output = opencv_estimateGeometricTransform(I1,I2)
endfunction
