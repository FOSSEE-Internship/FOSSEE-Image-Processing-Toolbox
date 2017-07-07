function classifierDescription = getParamsANN(Categoryclassifier)

Categoryclassifier_list = classifierToList(Categoryclassifier);
temp = raw_getParamsANN(Categoryclassifier_list);

classifierDescription = struct("bpms", temp(1), "bws", temp(2),"d0",temp(3),"d1",temp(4),"d2",temp(5),"d3",temp(6),"d4",temp(7),"nmeth",temp(8));


endfunction
