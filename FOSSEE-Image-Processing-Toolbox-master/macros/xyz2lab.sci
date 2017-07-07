function [output] = xyz2lab(vartype,varargin)
	[lhs rhs] = argn(0);
	if(rhs>3)
		error(msprintf("Too many input argumnets"));
	end
	
	select rhs
	case 1 then
		a = opencv_xyz2lab(vartype);
	case 2 then
		a = opencv_xyz2lab(vartype,varargin(0));
	case 3 then
		a = opencv_xyz2lab(vartype,varargin(0),varargin(1));
	end

	d = size(a);
	for i=1:d
		output(:,:,i) = a(i);
	end
endfunction

