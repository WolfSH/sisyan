function [ k ] = XLagerra(i,n)

k = ones(n+1,1);
%k = k-k;
k(:) = 0;

% if ( i == 0)
    % k = ones(size(x))/2;
% elseif ( i == 1)
    % k = x;
% else
    % A = PLagerra(i-1,x);
    % B = PLagerra(i-2,x);
    % k = (2*i-1-x).*A - ((i-1)^2).*B; 
% end
if (n>3)
	n = 3;
end
if (i>n)
	i = n;
end

switch i
    case 0 
		k(1) = 1;
	case 1
		k(1) = 0;
		k(2) = 1;
	case 2
		k(1) = 2;
		k(2) = -4;
		k(3) = 1;
	case 3
		k(1) = 6;
		k(2) = -18;
		k(3) = 9;
		k(4) = -1;
end