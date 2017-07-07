function [output] = ntsc2rgb(pstDataR,pstDataG,pstDataB)
	a = opencv_ntsc2rgb(pstDataR,pstDataG,pstDataB);
	d = size(a);
	for i=1:d
		output(:,:,i) = a(i);
	end
endfunction
