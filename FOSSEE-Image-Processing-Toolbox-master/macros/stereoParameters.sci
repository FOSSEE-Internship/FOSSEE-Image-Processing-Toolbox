function varargout=stereoParameters(varargin)
    [lhs rhs]=argn(0);
    if lhs<1 then
         error(msprintf(" Not enough output arguments. One expected"))
    elseif lhs>1 then
         error(msprintf(" Too many onput arguments. One expected"))
    end
    
    if rhs~=4 then
        error(msprintf(" Invalid number of input arguments"))    
    end
    
    EssentialMatrix = [];
    FundamentalMatrix = [];
    MeanReprojectionError = 0; 
    NumPatterns = 0;
    WorldPoints=[];
    WorldUnits='mm';
    
    cameraParam1 = varargin(1);
    cameraParam2 = varargin(2);

    if ~isstruct(cameraParam1) then
        error(msprintf(" A Camera Parameter object expected"))
    end
    
    if ~isstruct(cameraParam2) then
        error(msprintf(" A Camera Parameter object expected"))
    end
    
    xz = fieldnames(cameraParam1);
    [size1 zz] = size(xz);
    xz = fieldnames(cameraParam2);
    [size2 zz] = size(xz);    
      
    if size1~=18 | size2~=18 then
        error(msprintf(" Invalid Camera Parameter Object"))
    end   
    
    checkCameraParam(cameraParam1);
    checkCameraParam(cameraParam2);
    // checking third input
    
    rotationOfCam = varargin(3);
    [type3] = type(rotationOfCam);
    
    if type3~=1 | ~isreal(rotationOfCam) then
        error(msprintf(" Invalid Input Argument. A real 3x3 Matrix expected."))    
    end
    
    [rows cols] = size(rotationOfCam);
    
    if rows*cols~=9 then
        error(msprintf(" Invalid Input Argument. A 3x3 Matrix expected."))
    end
    
    // checking 4th input
    
    translationOfCam = varargin(4);
    [type4] = type(translationOfCam);
    
    if type4~=1 | ~isreal(translationOfCam) then
        error(msprintf(" Invalid Input Argument. A real list of 3 expected. "))
    end
    
    [rows cols] = size(translationOfCam);
    
    if rows*cols~=3 then
        error(msprintf("  Invalid Input Argument. A 3x3 Matrix expected."))
    end


    g = translationOfCam;
    gx =  [0 -g(3) g(2); g(3) 0 -g(1); -g(2) g(1) 0];
    EssentialMatrix = rotationOfCam*gx;
    FundamentalMatrix = cameraParam1.IntrinsicMatrix \ EssentialMatrix / (cameraParam2.IntrinsicMatrix)';
    MeanReprojectionError  = (cameraParam1.meanReprojectionError + cameraParam2.meanReprojectionError)/2;
    NumPatterns = cameraParam1.NumPatterns;
    WorldPoints = cameraParam1.WorldPoints;
    WorldUnits = cameraParam1.WorldUnits;
    
    varargout(1) = struct('CameraParameters1', cameraParam1, 'CameraParameters2', cameraParam2, 'RotationOfCamera2', rotationOfCam, 'TranslationOfCamera2', translationOfCam, 'FundamentalMatrix', FundamentalMatrix, 'EssentialMatrix', EssentialMatrix,'MeanReprojectionError', MeanReprojectionError, 'NumPatterns', NumPatterns, 'WorldPoints', WorldPoints, 'WorldUnits', WorldUnits);

endfunction

