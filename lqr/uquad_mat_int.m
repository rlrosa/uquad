function uquad_mat_int(A)
%Integrates A^t
n=length(A(:,1));
t=[1:5e-4:5e-2];
intA=0;
for i=1:n
    for j=1:n
       for k=1:length(t)     
        intA=intA+t(k)*A^t(k);
       end
    end
end
        