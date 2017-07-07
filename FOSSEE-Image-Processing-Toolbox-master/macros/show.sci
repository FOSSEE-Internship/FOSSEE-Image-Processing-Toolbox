     function[] =show(Img,ColorMap,winName,winSize)
     FigureHandle = gcf();
     drawlater();
     FigureHandle.color_map = ColorMap
     FigureHandle.background = -2; // sets the background to white
     FigureHandle.figure_name = winName;    
     [NumberOfRows NumberOfColumns] = size(Img);
     FigureHandle.figure_size=[int(winSize(1)),int(winSize(2))];
     FigureHandle.axes_size = [NumberOfColumns NumberOfRows];
     delete(gca()); // previous image is deleted
     Diagram = gca();
    // [NumberOfRows NumberOfColumns] = size(Img);
     Diagram.data_bounds = [1, 1; NumberOfColumns, NumberOfRows];
     Diagram.axes_visible = ['off' 'off' 'off'];
     Diagram.isoview = 'on';
     Options = '082'; // Box is drawn around image.
     Matplot(Img, Options);
     drawnow();
     endfunction
