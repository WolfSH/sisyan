function [ k ] = XErmit(i,n)

k = ones(n+1,1);
%k = k-k;
k(:) = 0;

% if ( i == 0)
    % k = ones(size(x))/2;
% elseif ( i == 1)
    % k = x;
% else
    % A = PErmit(i-1,x);
    % B = PErmit(i-2,x);
    % k = 2*x.*A - 2*(i-1).*B; 
% end

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
		k(1) = 0;
		k(2) = 2;
	case 2
		k(1) = -2;
		k(2) = 0;
		k(3) = 4;
	case 3
		k(1) = 0;
		k(2) = -12;
		k(3) = 0;
		k(4) = 8;
	case 4
		k(1) = 12;
		k(2) = 0;
		k(3) = -48;
		k(4) = 0;
		k(5) = 16;
end