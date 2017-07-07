function [output] = xyz2double(pstData)
	a = opencv_xyz2double(pstData);
	d = size(a);
	for i=1:d
		output(:,:,i) = a(i);
	end
endfunction
