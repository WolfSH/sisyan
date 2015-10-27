function [ k ] = PChebyshev(i,x)

if ( i == 0)
    k = ones(size(x))/2;
elseif ( i == 1)
    k = -1+2*x;
else
    A = PChebyshev(i-1,x);
    B = PChebyshev(i-2,x);
    k = (2.*(-1+2*x).*A - B); 
end