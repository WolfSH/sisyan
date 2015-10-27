function [ k ] = XChebyshev(i,n)

k = ones(n+1,1);
%k = k-k;
k(:) = 0;

    % A = XChebyshev(i-1,n);
    % B = XChebyshev(i-2,n);
    % for j=2:n
	%	C[j] = B[j-1];
	% end
    % k = (- 2.*A + 4.*C - B); 
if (n>4)
	n = 4;
end
if (i>n)
	i = n;
end

switch i
    case 0 
		k(1) = 1;
	case 1
		k(1) = -1;
		k(2) = 2;
	case 2
		k(1) = 1;
		k(2) = -8;
		k(3) = 8;
	case 3
		k(1) = -1;
		k(2) = 18;
		k(3) = -48;
		k(4) = 32;
	case 4
		k(1) = 1;
		k(2) = -32;
		k(3) = 160;
		k(4) = -256;
		k(5) = 128;
end