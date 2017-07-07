// Copyright (C) 2015 - IIT Bombay - FOSSEE
//
// This file must be used under the terms of the CeCILL.
// This source file is licensed as described in the file COPYING, which
// you should have received as part of this distribution.  The terms
// are also available at
// http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt
// Author: Gursimar Singh
// Organization: FOSSEE, IIT Bombay
// Email: toolbox@scilab.in
// Training Dataset credits : AT&T Laboratories Cambridge

function CreateSample(imgPath,positiveFile,negativeImages,varargin)
// It creates a .vec file from an image.The vec file contains the generated positive samples.The vec file is used for training a cascade classifier. 
//
// Calling Sequence
// CreateSample(imagePath,positiveFile,negativeImages)
// CreateSample(imagePath,positiveFile,negativeImages,"num",nsamples)
// CreateSample(imagePath,positiveFile,negativeImages,"w",width)
// CreateSample(imagePath,positiveFile,negativeImages,"h",height);
// CreateSample(imagePath,positiveFile,negativeImages,"num",nsamples,"w",width);
// CreateSample(imagePath,positiveFile,negativeImages,"num",nsamples,"w",width);
// CreateSample(imagePath,positiveFile,negativeImages,"h",height,"w",width);
// CreateSample(imagePath,positiveFile,negativeImages,"num",nsamples,"h",height,"w",width);
//
// Parameters
// imgPath: image path.Accepted file formats are .jpg,.jpeg,.png,.bmp,.pgm. 
// negativeImges:path to negativeImages.Accepted file formats are .jpg,.jpeg,.png,.bmp,.pgm.
// positiveFile : name of the output vec file.
// num:number of samples to be created.
// w: Width of training samples (in pixels). Default-25
// h: Height of training samples (in pixels). Default-25
//
// Description
// It produces dataset of positive  samples in a file with .vec extension and negative samples are enumerated in a special text file in
// which  each line contains an image filename of negative sample image. Negative images must not contain detected objects.
//
// By using these two files trainCascade will create cascade.xml file inside the outputFolder which is used to detect objects in an image.
//
// Examples
// CreateSample("images/positive.jpg","positive.vec","images/negative_samples","num",100);
// trainCascadeObjectDetect("positive_samples","images/negative_samples","positive.vec","numStages",6);
// im=imread("images/jellyfish.jpg");
// [image,bbox]=CascadeObjectDetector(im,"positive_samples/cascade.xml");
// imshow(image);
//
//See also
//trainCascadeObjectDetect
//CascadeObjectDetector
//
//Authors
//Gursimar Singh

    [lhs rhs]=argn(0);
    if rhs<3 then
         error(msprintf(" Not enough input arguments"))
    elseif rhs>9 then
         error(msprintf(" Too many input arguments to the function"))
    elseif modulo(rhs-1,2)     
         error(msprintf(" wrong number of input arguments,name-value pairs not macthed"))
    end
    
    //validating variables

    if ~isdir(negativeImages)
        error(msprintf(" wrong input argument #3,existing directory expected"))
    end
    
    //default values
    w=25;
    h=25;
    num=50;
    if rhs >3

        for i=1:2:rhs-3
           if strcmpi(varargin(i),"num")==0 then
                num=varargin(i+1);
                if num<0 then
                    error(msprintf(" num value must be positive"))
                end
            
                
            elseif strcmpi(varargin(i),'w')==0 then
                w=varargin(i+1);
                if h<0 then
                    error(msprintf(" w value must be positive"))
                end
                
            elseif strcmpi(varargin(i),'h')==0 then
                h=varargin(i+1);
                if h<0 then
                    error(msprintf(" h value must be positive"))
                end
            else
                error(msprintf(_(" Wrong value for input argument #%d",i)));
            end
        end
    end


if isdir(negativeImages)
        if getos()=="Linux"
            temp=strcat(["ls ",negativeImages])
        elseif getos()=="Windows"
            temp=strcat(["dir ",negativeImages])
        end
        s=unix_g(temp);
        disp(temp)
        [noOfFilesInFolder noOfCols]=size(s);
        fd = mopen('negative.txt','wt');
        for i=1:noOfFilesInFolder
            [path,fname,extension]=fileparts(s(i))
            if ~strcmp(extension,".jpg") | ~strcmp(extension,".jpeg") | ~strcmp(extension,".png") | ~strcmp(extension,".bmp") | ~strcmp(extension,".pgm") | ~strcmp(extension,".JPG") | ~strcmp(extension,".JPEG") | ~strcmp(extension,".PNG") | ~strcmp(extension,".BMP") | ~strcmp(extension,".PGM")
                mfprintf(fd,'%s/%s\n',negativeImages,s(i));
            end
        end
     end
    mclose(fd);
    disp("Creating positive samples:");
    cmd=sprintf("opencv_createsamples -img %s -bg negative.txt -num %d -vec %s -w %d -h %d ",imgPath,num,positiveFile,w,h);
    disp(cmd);
    unix_w(cmd);
    
endfunction
