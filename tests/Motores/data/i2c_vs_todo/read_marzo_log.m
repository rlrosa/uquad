function [log1,log2,log3,log4] = read_marzo_log()

F    = fopen('tests/Motores/data/i2c_vs_todo/notas_cuad_marzo.txt');
Daux = textscan(F,'%s','delimiter','\t');
fclose(F);

j = 1;
for i=1:length(Daux{1})
    if Daux{1}{i}(1)~='%'
      D{1}{j,1}=Daux{1}{i};
      j=j+1;
    end
end

cols = 3;
a=str2double(D{1}(1:cols:end));
b=str2double(D{1}(2:cols:end));
c=str2double(D{1}(3:cols:end));

log4 = [a(1:9) c(1:9) b(1:9)];
log3 = [a(10:18) c(10:18) b(10:18)];
log2 = [a(19:27) c(19:27) b(19:27)];
log1 = [a(28:35) b(28:35) c(28:35)];

figure();
    plot(log1(:,1),log1(:,2),'b-*')
    hold on
    plot(log2(:,1),log2(:,2),'r-*')
    plot(log3(:,1),log3(:,2),'g-*')
    plot(log4(:,1),log4(:,2),'k-*')
    grid; legend('log1','log2','log3','log4')
    title('i2c vs fuerza'); xlabel('i2c'); ylabel('fuerza');
    
figure();
    plot(log1(:,1),pi*log1(:,3),'b-*')
    hold on
    plot(log2(:,1),pi*log3(:,3),'r-*')
    plot(log4(:,1),pi*log4(:,3),'k-*')
    grid; legend('log1','log3','log4')
    title('i2c vs w'); xlabel('i2c'); ylabel('w');
    
% figure();
%     plot(log1(:,3),'b-*')
%     hold on
%     plot(log2(:,3),'r-*')
% %     plot(log3(:,3),'g-*')
%     plot(log4(:,3),'k-*')
%     grid; legend('log1','log2','log4')
%     title('w'); xlabel('muestra'); ylabel('w');
    