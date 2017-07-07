function classifierDescription = getParamsBoost(Categoryclassifier)

Categoryclassifier_list = classifierToList(Categoryclassifier);
temp = raw_getParamsBoost(Categoryclassifier_list);

classifierDescription = struct("bt", temp(1), "wc", temp(2),"wtr",temp(3),"md",temp(4),"use_surr",temp(5));


endfunction
