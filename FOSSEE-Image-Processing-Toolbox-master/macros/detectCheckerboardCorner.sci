function[corners]=detectCheckerboardCorner(image,patternSize,varargin)

image_list = mattolist(image)
//Categoryclassifier_list = classifierToList(Categoryclassifier);

[lhs,rhs]=argn(0);

if lhs>1
    error(msprintf(" Too many output arguments"));
	elseif rhs>3
    error(msprintf(" Too many input arguments"));
	elseif rhs<2
   	error(msprintf("Too less arguments provided!,minimum is 2!"));
	end

	if rhs==2 then
	corners = raw_detectCheckerboardCorner(image_list,patternSize);
	end
	if rhs==3 then
	corners = raw_detectCheckerboardCorner(image_list,patternSize,varargin(1));
	end
endfunction
