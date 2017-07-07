function [output] = lab2uint8(pstData)
	a = opencv_lab2uint8(pstData);
	d = size(a);
	for i=1:d
		output(:,:,i) = a(i);
	end
endfunction
