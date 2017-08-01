// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in

function[] =imshow(Image,varargin)         
// Used to view an image
//
// Calling Sequence
// src = imshow(Image);
// src = imshow(Image,winName);
// src = imshow(Image,winName,winSize);
// 
// Parameters
// Image : Input image
// winName= Name of the image window specified as a string.
// winSize=Size of the image window specified as [rows,cols]
//
// Examples
// src = imread('images/lena.jpeg');
// imshow(src);

      
                                  
    Image=mattolist(Image);   //retrieving list and creating 3 dimensional matrix out of it     
    dimensions=size(Image)   //convert hyper matrix to list  

    [lhs rhs]= argn(0);
    
    if lhs>1
       error(msprintf("Too many outut arguments"));
    end
    if rhs<1
       error(msprintf("Not enough input arguments"));
    end

    if dimensions==3 then 
     [c d]=size(Image(1));
     r=matrix(Image(1),c,d);
     g=matrix(Image(2),c,d);
     b=matrix(Image(3),c,d);
     z(:,:,1)=r; 
     z(:,:,2)=g; 
     z(:,:,3)=b;
     [NumberOfRows NumberOfColumns NumberOfChannels] = size(z);
     winSize=[(1080/NumberOfRows)*NumberOfRows,(1920/NumberOfColumns)*NumberOfColumns];
     NumberOfPixels = NumberOfRows * NumberOfColumns;
     MaxGrayValue = 2 ^ 8 - 1;
     ColorMap = double(matrix(z, NumberOfPixels, NumberOfChannels)) ...
           / MaxGrayValue;
     Img = matrix(1 : NumberOfPixels, NumberOfRows, NumberOfColumns);
      
     elseif dimensions==1 then                                 // its a single channel image
        MaxUInt8 = 8;                                              //declaring MaxUInt8 
            
        depth = getDepth(Image(1));
        if (strcmp(depth,'CV_16U')==0 | strcmp(depth,'CV_64F')==0 ) //if its a 16 bit image or a 64 bit float image
        Image(1) = convert(Image(1),'CV_8U',255,0);          //we'll convert it to 8 bit unsigned int 
                                                            // if its a 64 bit float image, we assume that 
                                                           //it has normalized for the range [0,1]                                           
        end 
        
            
        [c d]=size(Image(1));
        Img=matrix(Image(1),c,d);
        [NumberOfRows NumberOfColumns]=size(Img);
        winSize=[(1080/NumberOfRows)*NumberOfRows,(1920/NumberOfColumns)*NumberOfColumns];
        MaximumGrayValue = 2^MaxUInt8 -1;
        ColorMap = graycolormap(double(MaximumGrayValue + 1));
    end;
     winName="Title";
     if rhs>1 then
        winName=varargin(1);
        if type(winName) ~= 10
            error(msprintf("Figure Name must be a string"));
        end 
        if rhs==3 
            winSize=varargin(2);
        end        
     end   
     show(Img,ColorMap,winName,winSize);
endfunction

