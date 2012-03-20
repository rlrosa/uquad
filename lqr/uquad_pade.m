function expA=uquad_pade(A)

j=2;
m=2^j;
n=length(A(:,1));

A=A/m;

X=A;
c=1;
E=eye(n);
D=eye(n);

q=1;

for k=1:q
    
    c=c*(q-k+1)/(k*(2*q-k+1));
    X=A*X;
    E=E+c*X;
    if (floor(k/2)*2==k)
        D=D+c*X;
    else
        D=D-c*X;
    end
    
end

E=D\E;

expA=eye(n);

for i=1:m
    expA=expA*E;
end