function [archivos,N]=cargo_nombres_gyro()

%% QUIETO

str_cont=1;
str=['gyro/logs/x00y00'];
archivos{str_cont}=str;
str_cont=str_cont+1;


%% TALADRO

for i=1:3
    if i==1
        eje='x';
        otro='y';
    elseif i==2
        eje='y';
        otro='z';
    else
        eje='z';
        otro='x';
    end
    for j=1:2
        if j==1
           vel='v1';
        elseif j==2
            vel='v2';
        end        
        for k=2:2                   % OJO
            if k==1
                theta='00';
            else
                theta='30';
            end
            str=['gyro/logs/' eje vel otro theta];            
            archivos{str_cont}=str;
            str_cont=str_cont+1;
        end
    end
end


%% TOCADISCOS

vel='v3';

for i=1:3
    if i==1
        eje='x';
        otro='z';
    elseif i==2
        eje='y';
        otro='x';
    else
        eje='z';
        otro='y';
    end
    for k=1:1                           % OJO
        if k==1
            theta='00';
        elseif k==2
            theta='30';
        else
            theta='45';
        end
        str=['gyro/logs/' eje vel otro theta];            
        archivos{str_cont}=str;
        str_cont=str_cont+1;
    end

end




str=['gyro/logs/zv3y30'];       % OJO
archivos{str_cont}=str;



N=size(archivos,2);