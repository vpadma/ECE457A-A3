function [ Z ] = sixhumpcamelback(X, Y)
%sixhumpcamelback The objective function of the PSO

Z = (4-2.1*(X.^2)+((X.^4)./3)).*(X.^2)+(X.*Y)+(-4+4*Y.^2).*(Y.^2);

end

