function w=calc_omega(F)

syms x
val=solve(4.60160135072435e-05*x^2-0.00103822726273726*x-F);
val_hov=eval(val);
        
if (val_hov(1)>=0)
     w=val_hov(1);
else w=val_hov(2);
end