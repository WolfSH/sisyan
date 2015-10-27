function [ k ] = PLagerra(i,x)

if ( i == 0)
    k = ones(size(x))/2;
elseif ( i == 1)
    k = x;
else
    A = PLagerra(i-1,x);
    B = PLagerra(i-2,x);
    k = (2*i-1-x).*A - ((i-1)^2).*B; 
end
