function [output] = lab2rgb(pstData)
	a = opencv_lab2rgb(pstData);
	d = size(a);
	for i=1:d
		output(:,:,i) = a(i);
	end
endfunction
