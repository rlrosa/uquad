function expA=uquad_exp(A);

n=length(A(:,1));

I=eye(n);

expA=I;
e=zeros(n);
k=1;
fact=1;
while norm(e-expA)>1e-5
    e=expA;
    expA=expA+(A^k)/fact;
    k=k+1;
    fact=fact*k;
end
