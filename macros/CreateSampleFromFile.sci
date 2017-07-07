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

function CreateSampleFromFile(positiveImages,positiveFile,num,varargin)
// It creates a .vec file from a set of positive images.The vec file is used for training a cascade classifier. 
//
// Calling Sequence
// CreateSampleFromFile(positiveImages,positiveFile,num);
// CreateSampleFromFile(positiveImages,positiveFile,num,"w",width);
// CreateSampleFromFile(positiveImages,positiveFile,num,"h",height);
// CreateSampleFromFile(positiveImages,positiveFile,num,"recursive");
// CreateSampleFromFile(positiveImages,positiveFile,num,"w",width,"h",height);
// CreateSampleFromFile(positiveImages,positiveFile,num,"w",width,"h",height,"recursive");
//
// Parameters
// positiveImages: positive images folder path.Accepted file formats are .jpg,.jpeg,.png,.bmp,.pgm.
// positiveFile: name of the output vec file.
// num: Number of samples to be generated.
// w: Width of training samples (in pixels). Default-25
// h: Height of training samples (in pixels). Default-25
//"reccursive": If there exists sublofolders in positive image folder.
//
// Description
// It produces dataset of positive  samples in a file with .vec extension.The vec file is used for training a cascade classifier.  
//
// Examples
//     //The example detects faces in an image by training a cascade classifier using user defined dataset of 400 images. 
// CreateSampleFromFile("images/trainset_face","positive_faces.vec",400);
// trainCascadeObjectDetect("positive_samples_faces_demo","negative_samples","positive_faces.vec","numPos",300,"numStages",5);
// im=imread("images/faces.jpg");
// img=CascadeObjectDetector(im,"positive_samples_faces/cascade.xml",1.04,1,8);
// imshow(img);
//
// Examples
//   //Using the image dataset which contains subfolders within the positiveImages directory. 
// CreateSampleFromFile("images/trainset_face","positive_faces.vec",400,"recursive");
// trainCascadeObjectDetect("positive_samples_faces","images/negative_samples","positive_faces.vec","numPos",300,"numStages",5);
// //number of positive samples used in trainCascade Object Detect must not be more than 90% of total numver of poitive images in vec file
// im=imread("images/faces.jpg");
// img=CascadeObjectDetector(im,"positive_samples_faces/cascade.xml",1.04,1,8);
// imshow(img);
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
    elseif rhs>8 then
         error(msprintf(" Too many input arguments to the function"))
    end
    

    
    //default values
    w=25;
    h=25;
    recurr=0;
    if rhs >3

        for i=1:1:rhs-3
            
            if strcmpi(varargin(i),'w')==0 then
                i=i+1;
                w=varargin(i);
                if h<0 then
                    error(msprintf(" w value must be positive"))
                end
                
            elseif strcmpi(varargin(i),'h')==0 then
                i=i+1;
                h=varargin(i);
                if h<0 then
                    error(msprintf(" h value must be positive"))
                end

             elseif (strcmpi(varargin(i),'recursive')==0) then 
                recurr=1;  
            else
                error(msprintf(_(" Wrong value for input argument #%d",i)));
            end
        end
    end

     
    if (recurr==1)

        folders=listfiles(positiveImages);
        noOfFolders=size(folders);
        disp(noOfFolders(1));
        fd = mopen('positive.txt','wt');
        for j=1:noOfFolders(1)
            if isdir(positiveImages)
                if getos()=="Linux"
                    temp=strcat(["ls ",positiveImages,"/",folders(j)])
                elseif getos()=="Windows"
                    temp=strcat(["dir ",positiveImages,"\",folders(j)])
                end
                s=unix_g(temp);
                [noOfFilesInFolder noOfCols]=size(s);
                
                //disp(noOfFilesInFolder);
                disp(folders(j));
                for i=1:noOfFilesInFolder
                    [path,fname,extension]=fileparts(s(i))
                    if ~strcmp(extension,".jpg") | ~strcmp(extension,".jpeg") | ~strcmp(extension,".png") | ~strcmp(extension,".bmp") | ~strcmp(extension,".pgm") | ~strcmp(extension,".JPG") | ~strcmp(extension,".JPEG") | ~strcmp(extension,".PNG") | ~strcmp(extension,".BMP") | ~strcmp(extension,".PGM")
                    //disp(s(i));
                        mfprintf(fd,'%s/%s/%s 1',positiveImages,folders(j),s(i));
                        im=imread(positiveImages + "/" + folders(j) + "/" +s(i));
                    sz=size(im);
                    boxVals=[0 0 sz(2),sz(1)];
                    for j=1:4
                        mfprintf(fd,' %d',boxVals(j));
                    end
                    mfprintf(fd,'\n');
                    end
                end
             end
        end
        mclose(fd);    
       
    else
         fd = mopen('positive.txt','wt');
         if isdir(positiveImages)
                if getos()=="Linux"
                    temp=strcat(["ls ",positiveImages])
                elseif getos()=="Windows"
                    temp=strcat(["dir ",positiveImages])
                end
                s=unix_g(temp);
                [noOfFilesInFolder noOfCols]=size(s);
                
                
              
                for i=1:noOfFilesInFolder
                    [path,fname,extension]=fileparts(s(i))
                    if ~strcmp(extension,".jpg") | ~strcmp(extension,".jpeg") | ~strcmp(extension,".png") | ~strcmp(extension,".bmp") | ~strcmp(extension,".pgm") | ~strcmp(extension,".JPG") | ~strcmp(extension,".JPEG") | ~strcmp(extension,".PNG") | ~strcmp(extension,".BMP") | ~strcmp(extension,".PGM")
                    //disp(s(i));
                        mfprintf(fd,'%s/%s 1',positiveImages,s(i));
                        im=imread(positiveImages + "/"  +s(i));
                    sz=size(im);
                    boxVals=[0 0 sz(2),sz(1)];
                    for j=1:4
                        mfprintf(fd,' %d',boxVals(j));
                    end
                    mfprintf(fd,'\n');
                    end
                end
        end

    end  
  
    
    
    disp("Creating positive samples:");
    cmd=sprintf("opencv_createsamples -info positive.txt -num %d -vec %s -w %d -h %d ",num,positiveFile,w,h);
    unix_w(cmd);
    
endfunction
