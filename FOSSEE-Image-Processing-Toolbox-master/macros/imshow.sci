function[] =imshow(Image,varargin)         //retrieving list and creating 3 dimensional matrix out of it
//varargin(1)-"figure_name"
//varargin(2)=[Rows,Cols] 
    Image=mattolist(Image);       //convert hyper matrix to list  
    dimensions=size(Image)
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
     elseif dimensions==1 then
     [c d]=size(Image(1));
     Img=matrix(Image(1),c,d);
     [NumberOfRows NumberOfColumns]=size(Img);
     winSize=[(1080/NumberOfRows)*NumberOfRows,(1920/NumberOfColumns)*NumberOfColumns];
     MaxUInt8 = 2 ^ 8 - 1;
     MaximumGrayValue = MaxUInt8;
     ColorMap = graycolormap(double(MaximumGrayValue + 1));
     end;
     winName="Title";
     if rhs>2 then
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
