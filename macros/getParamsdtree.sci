function classifierDescription = getParamsdtree(Categoryclassifier)

Categoryclassifier_list = classifierToList(Categoryclassifier);
temp = raw_getParamsdtree(Categoryclassifier_list);

classifierDescription = struct("cv_f", temp(1), "max_cat", temp(2),"md",temp(3),"msc",temp(4),"reg_acc",temp(5),"prune",temp(6),"use1_se",temp(7),"use_surr",temp(8));


endfunction
