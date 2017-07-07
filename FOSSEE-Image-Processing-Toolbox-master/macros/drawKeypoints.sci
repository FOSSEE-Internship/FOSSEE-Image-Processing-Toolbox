function [x] = drawKeypoints(img,keypoints,varargin)
//Calling Sequence 
//drawKeypoints(img,keypoints)
//rawKeypoints(img,keypoints,"color",[0,255,0]);
//variable inputs : match color string & name , flags string and name  
img_list=mattolist(img);
 [lhs rhs] = argn(0);
     if rhs>6 then
         error(msprintf("Too many input arguments"));
     end
     if rhs<2 then
	 error(msprintf("Not enough input arguments"));
	 end
	 if lhs >1
	 error(msprintf("Too many output arguments"));
     end

if rhs==2
y=raw_drawKeypoints(img_list,keypoints);
elseif rhs==4 then
y=raw_drawKeypoints(img_list,keypoints,varargin(1),varargin(2));
elseif rhs==6 then
y=raw_drawKeypoints(img_list,keypoints,varargin(1),varargin(2),varargin(3),varargin(4));
else
error(msprintf("Invalid argument format"));
end

//x=y;
channel = size(y);
	
	for i = 1:channel
		x(:,:,i) = y(i);
			end
endfunction