function [output] = lab2double(pstData)
	a = opencv_lab2double(pstData);
	d = size(a);
	for i=1:d
		output(:,:,i) = a(i);
	end
endfunction
