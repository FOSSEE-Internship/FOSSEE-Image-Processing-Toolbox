function [output] = ind2gray(img,colormap)
	image = mattolist(img);
	a = opencv_ind2gray(image,colormap)
	d = size(a);
	for i=1:d
		output(:,:,i) = a(i);
	end
endfunction
