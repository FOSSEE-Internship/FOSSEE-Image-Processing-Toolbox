function [output] = imrect(img,pstDataX,pstDataY)
	image = mattolist(img);
	a = opencv_imrect(image,pstDataX,pstDataY)
	d = size(a);
	for i=1:size
		output(:,:,i) = a(i);
	end
endfunction
