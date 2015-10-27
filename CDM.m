function [out] = CDM (A0, b0)

% Приведение к квадратному виду
A = A0' * A0;
b = A0' * b0;
m = length (A);

eps = 1;
x = zeros (m, 1);
d = zeros (m, 1);
g = -b;

while abs (eps) > 0.0001
    g_prev = g;
    g = A * x - b;
    d = -g + ((g' * g) / (g_prev' * g_prev)) * d;
    s = - (d' * g) / (d' * A * d);
    x = x + s * d;
    eps = norm (A * x - b);
end

out = x;