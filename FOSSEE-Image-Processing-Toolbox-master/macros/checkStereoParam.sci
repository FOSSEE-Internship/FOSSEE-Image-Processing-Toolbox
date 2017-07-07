function checkStereoParam(varargin)
    stereoParam = varargin(1);
    
    if ~isfield(stereoParam, 'CameraParameters1') | ~isfield(stereoParam,'CameraParameters2') | ~isfield(stereoParam, 'RotationOfCamera2') | ~isfield(stereoParam, 'TranslationOfCamera2') | ~isfield(stereoParam, 'FundamentalMatrix') | ~isfield(stereoParam, 'EssentialMatrix') | ~isfield(stereoParam, 'MeanReprojectionError') | ~isfield(stereoParam, 'NumPatterns') | ~isfield(stereoParam, 'WorldPoints') | ~isfield(stereoParam, 'WorldUnits')  then
            error(msprintf("A Stereo Paramter struct expected."))
    end
    
    cam1 = stereoParam.CameraParameters1;
    cam2 = stereoParam.CameraParameters2;
    checkCameraParam(cam1);
    checkCameraParam(cam2);
    
    rotCam = stereoParam.RotationOfCamera2;
    transCam = stereoParam.TranslationOfCamera2;
    funMat = stereoParam.FundamentalMatrix;
    essenMat = stereoParam.EssentialMatrix;
    mre = stereoParam.MeanReprojectionError;
    numPat = stereoParam.NumPatterns;
    worldPoints  = stereoParam.WorldPoints;
    worldUnits = stereoParam.WorldUnits;
    
    if type(rotCam)~=1 | type(transCam)~=1 | type(funMat)~=1 | type(essenMat)~=1 | type(mre)~=1 | type(numPat)~=1 | type(worldPoints)~=1 | type(worldUnits)~=10 | ~isreal(rotCam) | ~isreal(transCam) | ~isreal(funMat) | ~isreal(essenMat) | ~isreal(mre) | ~isreal(numPat) | ~isreal(worldPoints) then
        error(msprintf("A Stereo Paramter struct expected."))
    end
    
    [size11 size12] = size(rotCam);
    [size21 size22] = size(transCam);
    
    if (size11*size12)~=9 | (size21*size22)~=3 then
        error(msprintf("A Stereo Paramter struct expected."))
    end
   
endfunction
