function [out] = fillconvexpoly(img,pstData,npts,r_value,g_value,b_value,linetype,shift)
	image = mattolist(img);
	a = opencv_fillconvexpoly(image,pstData,npts,r_value,g_value,b_value,linetype,shift)
	d = size(a);
	for i=1:d
		out(:,:,i) = a(i);
	end
endfunction
