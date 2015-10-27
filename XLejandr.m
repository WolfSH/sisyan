function [ k ] = XLejandr(i,n)

k = ones(n+1,1);
%k = k-k;
k(:) = 0;

% if ( i == 0)
    % k = ones(size(x))/2;
% elseif ( i == 1)
    % k = x;
% else
    % A = PLejandr(i-1,x);
    % B = PLejandr(i-2,x);
    % k = ((2*i-1).*x.*A - (i-1).*B ) / i; 
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
		k(1) = -0.5;
		k(2) = 0;
		k(3) = 1.5;
	case 3
		k(1) = 0;
		k(2) = -1.5;
		k(3) = 0;
		k(4) = 2.5;
end