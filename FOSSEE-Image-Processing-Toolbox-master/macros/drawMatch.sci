function [x] = drawMatch(img1,img2,keyPoints1,keyPoints2,indexPairs,distance,varargin)
//variable inputs : match color string & name , flags string and name  
img_list1=mattolist(img1);
img_list2=mattolist(img2);
 [lhs rhs] = argn(0);
     if rhs>10 then
         error(msprintf("Too many input arguments"));
     end
     if rhs<6 then
	 error(msprintf("Not enough input arguments"));
	 end
	 if lhs >1
	 error(msprintf("Too many output arguments"));
     end

if rhs==6
y=raw_drawMatch(img_list1,img_list2,keyPoints1,keyPoints2,indexPairs,distance);
elseif rhs==8 then
y=raw_drawMatch(img_list1,img_list2,keyPoints1,keyPoints2,indexPairs,distance,varargin(1),varargin(2));
elseif rhs==10 then
y=raw_drawMatch(img_list1,img_list2,keyPoints1,keyPoints2,indexPairs,distance,varargin(1),varargin(2),varargin(3),varargin(4));
else
error(msprintf("Invalid argument format"));
end

channel = size(y);
	
	for i = 1:channel
		x(:,:,i) = y(i);
	end
endfunction
