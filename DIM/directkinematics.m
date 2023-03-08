function xt = directkinematics(lx, q)
methods = dynlib;
xt = methods.dk(lx, q);
end