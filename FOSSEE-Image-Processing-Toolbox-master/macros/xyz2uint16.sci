function [output] = xyz2uint16(pstData)
	a = opencv_xyz2uint16(pstData);
	d = size(a);
	for i=1:d
		output(:,:,i) = a(i);
	end
endfunction
