function [] = print_like_uquad_mat(struct,nombre_matriz,A)
[M,N] = size(A);

for i=1:M
    for j=1:N
        fprintf('%s -> %s -> m[%d][%d] = %0.20f;\n',struct,nombre_matriz,i-1,j-1,A(i,j));
    end
end