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
// Training dataset credits AT&T Laboratories Cambridge

function trainCascadeObjectDetect(outputFolder,negativeImages,positiveFile,varargin)
// It creates trained cascade XML file which can be use for object/face detection. 
//
// Calling Sequence
// trainCascadeObjectDetect(outputFolder,negativeImages,positiveFile)
// trainCascadeObjectDetect(outputFolder,negativeImages,positiveFile,"numStages",numStages);
// trainCascadeObjectDetect(outputFolder,negativeImages,positiveFile,"numStages",numStages,"featureType",featureType);
// trainCascadeObjectDetect(outputFolder,negativeImages,positiveFile,"numStages",numStages,"featureType",featureType,"numPos",numPos);
// trainCascadeObjectDetect(outputFolder,negativeImages,positiveFile,"numStages",numStages,"featureType",featureType,"numPos",numPos,"numNeg",numNeg);
// trainCascadeObjectDetect(outputFolder,negativeImages,positiveFile,"numStages",numStages,"featureType",featureType,"numPos",numPos,"numNeg",numNeg,"nsplits",nsplits);
// trainCascadeObjectDetect(outputFolder,negativeImages,positiveFile,"numStages",numStages,"featureType",featureType,"numPos",numPos,"numNeg",numNeg,"nsplits",nsplits,"minHitRate",minHitRate);
// trainCascadeObjectDetect(outputFolder,negativeImages,positiveFile,"numStages",numStages,"featureType",featureType,"numPos",numPos,"numNeg",numNeg,"nsplits",nsplits,"minHitRate",minHitRate,"maxFalseAlarmRate",maxFalseAlarmRate);
// trainCascadeObjectDetect(outputFolder,negativeImages,positiveFile,"numStages",numStages,"featureType",featureType,"numPos",numPos,"numNeg",numNeg,"nsplits",nsplits,"minHitRate",minHitRate,"maxFalseAlarmRate",maxFalseAlarmRate,"w",width);
// trainCascadeObjectDetect(outputFolder,negativeImages,positiveFile,"numStages",numStages,"featureType",featureType,"numPos",numPos,"numNeg",numNeg,"nsplits",nsplits,"minHitRate",minHitRate,"maxFalseAlarmRate",maxFalseAlarmRate,"w",width,"h",height);
//
// Parameters
// outputFolder: Folder name to store trained cascade (cascade.xml) and intermediate files  
// negativeImages: path to a negative images folder.Accepted file formats are .jpg,.jpeg,.png,.bmp,.pgm.
// positiveFile: path to the vector file which contains sample data. 
// Name-Value Pair arguments:
// numPos: number of positive samples. Default-30
// numNeg: number of negative samples. Default- 20
// nsplits: number of splits. Default- 2
// numStages: number of cascade stages to be trained. Default- 30
// featureType: Type of features to be used, possible types are HAAR, LBP, HOG. Default-HAAR
// minHitRate: Minimal desired hit rate for each stage of the classifier and value in the range 0 and 1 inclusive. Default- 0.995
// maxFalseAlarmRate: Maximal desired false alarm rate for each stage of the classifier and value in the range 0 and 1 inclusive. Default- 0.5
// w: Width of training samples (in pixels). Default-25
// h: Height of training samples (in pixels). Default-25
//
// Description
// By using postiveFile.vec and negative.txt created using negativeImages folder, trainCascade will create cascade.xml file inside the outputFolder which is used to detect objects in an image.
//
// Examples
// //Training a obejct classifier using one positive image. 
// CreateSample("images/positive.jpg","positive.vec","images/negative_samples","num",100);
// trainCascadeObjectDetect("positive_samples","images/negative_samples","positive.vec","numStages",6);
// im=imread("images/jellyfish.jpg");
// [image,bbox]=CascadeObjectDetector(im,"positive_samples/cascade.xml");
// imshow(image);
//
// Examples
//   //Training a obejct classifier using the image dataset which contains subfolders within the positiveImages directory. 
// CreateSampleFromFile("images/trainset_face","positive_faces.vec",400,"recursive");
// trainCascadeObjectDetect("positive_samples_faces","images/negative_samples","positive_faces.vec","numPos",300,"numStages",5);//number of positive samples used in trainCascade Object Detect must not be more than 90% of total number of poitive images in vec file
// im=imread("images/faces.jpg");
// img=CascadeObjectDetect(im,"positive_samples_faces/cascade.xml",1.04,1,8,[50,50],[150,150]);
// imshow(img);
//
//See also
//CreateSampleFromFile
//CascadeObjectDetect
//
//Authors
//Gursimar Singh

    [lhs rhs]=argn(0);
    if rhs<3 then
         error(msprintf(" Not enough input arguments"))
    elseif rhs>21 then
         error(msprintf(" Too many input arguments to the function"))
    elseif modulo(rhs-3,2)
         error(msprintf(" wrong number of input arguments,name-value pairs not macthed"))
    end
    
    //validating variables
    
    if ~isdir(negativeImages)
        error(msprintf(" wrong input argument #3,existing directory expected"))
    end
    
    //default values
    numPos=20;
    numNeg=20;
    numStages=10;
    nsplits=2;
    featureType="HAAR"
    minHitRate=0.995
    maxFalseAlarmRate=0.5
    w=25
    h=25
    
    for i=1:2:rhs-3
       if strcmpi(varargin(i),"numPos")==0 then
            i=i+1;
            numPos=varargin(i);
            if numPos<0 then
                error(msprintf(" numPos value must be positive"))
            end
            disp(numPos);
        elseif strcmpi(varargin(i),'numNeg')==0 then
            i=i+1;
            numNeg=varargin(i);
            if numNeg<0 then
                error(msprintf(" numNeg value must be positive"))
            end
            
        elseif strcmpi(varargin(i),'numStages')==0 then
            i=i+1;
            numStages=varargin(i);
            if numStages<0 then
                error(msprintf(" numStages value must be positive"))
            end
            
        elseif strcmpi(varargin(i),'nslits')==0 then
            i=i+1;
            nsplits=varargin(i);
            if nsplits<0 then
                error(msprintf(" nsplits value must be positive"))
            end
            
        elseif strcmpi(varargin(i),'featureType')==0 then
            i=i+1;
            featureType=varargin(i);
            if strcmpi(featureType,'haar') & strcmpi(featureType,'lbp') & strcmpi(featureType,'hog')
                error(msprintf(" wrong input argument #%d,featureType not matched",i));
            end
            
        elseif strcmpi(varargin(i),'minHitRate')==0 then
            i=i+1;
            minHitRate=varargin(i);
            if minHitRate<0 | minHitRate>1 then
                error(msprintf(" minHitRate value must lie in between 0 and 1"))
            end
            
        elseif strcmpi(varargin(i),'maxFalseAlarmRate')==0 then
            i=i+1;
            maxFalseAlarmRate=varargin(i);
            if maxFalseAlarmRate<0  | minFalseRate>1 then
                error(msprintf(" maxFalseAlarmRate value must lie in between 0 and 1"))
            end
            
        elseif strcmpi(varargin(i),'w')==0 then
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
        else
            error(msprintf(_(" Wrong value for input argument #%d",i)));
        end
    end
    if ~isfile('negative.txt')
        if isdir(negativeImages)
            if getos()=="Linux"
                temp=strcat(["ls ",negativeImages])
            elseif getos()=="Windows"
                temp=strcat(["dir ",negativeImages])
            end
            s=unix_g(temp);
            [noOfFilesInFolder noOfCols]=size(s);
            fd = mopen('negative.txt','wt');
            for i=1:noOfFilesInFolder
                [path,fname,extension]=fileparts(s(i))
                if ~strcmp(extension,".jpg") | ~strcmp(extension,".jpeg") | ~strcmp(extension,".png") | ~strcmp(extension,".bmp") | ~strcmp(extension,".pgm") | ~strcmp(extension,".JPG") | ~strcmp(extension,".JPEG") | ~strcmp(extension,".PNG") | ~strcmp(extension,".BMP") | ~strcmp(extension,".PGM")
                    mfprintf(fd,'%s/%s\n',negativeImages,s(i));
                end
            end
           mclose(fd); 
         end
    end    

if isdir(outputFolder)
    removedir(outputFolder);
end    
    createdir(outputFolder);

disp("Training Cascade:");
     cmd=sprintf("opencv_traincascade -data %s -vec %s -bg negative.txt -numPos %d -numNeg %d -numStages %d -nsplits %d -featureType %s -minHitRate %d -maxFalseAlarmRate %d -w %d -h %d",outputFolder,positiveFile,numPos,numNeg,numStages,nsplits,featureType,minHitRate,maxFalseAlarmRate,w,h);
     unix_w(cmd);


endfunction 
