function [ k ] = PErmit(i,x)

if ( i == 0)
    k = ones(size(x))/2;
elseif ( i == 1)
    k = x;
else
    A = PErmit(i-1,x);
    B = PErmit(i-2,x);
    k = 2*x.*A - 2*(i-1).*B; 
end