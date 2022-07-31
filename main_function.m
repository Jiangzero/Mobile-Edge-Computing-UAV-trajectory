%main function
clear;
clc;
%variable definition
K = 6;%�û�����
N = 48; %ʱ϶����48
H = 100; %���˻����и߶�
T = 25; %ʱ������ 25
tt = T/(N+2); %ʱ϶����
b_0 = 1e-6;  %1m�ŵ�����-70dB
P_0 = 3e-3; %�û����͹���
P_h=59.2;%UAV0������������
f_max = 5e8;  %UAV0������Ƶ��
B_m = 1e8;%UAV1����1MHz
B_k = 5e7;%�û�����
np = 1e-14;  %�������� -110dBm=9.9e-15
C = 1e3;%ִ��һ��λ����CPU������
[Q,V] = tra(N,tt);%��ʼ���켣���ٶ�
%[Q,V] =UAV1();
q_0 = [0;100];%UAV0�ĳ�ʼλ��
W=[-20,60, 0, -40,-75,-50;
    45,100,170,160,115,50];
E = zeros(5,80);%��Ȩ���ܺ�
E_m = zeros(5,80);%��ʵ���ܺ�
L_E = zeros(5,80);%��λ������������û���Ϣ������������������
E_1 = zeros(5,10);%��Ȩ���ܺ�
E_F_1 = zeros(5,10);%��Ȩ���ܺ�
E_m_1 = zeros(5,10);%��ʵ���ܺ�
L_E_1 = zeros(5,10);%��λ������������û���Ϣ������������������
E_MC = zeros(5,1);
c = 0;
arig = 2;
i=1;
E_F = [192.28,213.5295,229.6308,254.3649,2794];
for T = 19:3:31
    tt = T/(N+2);
    E_MC(i) = metekaluo(K,N,E_F(i),tt);
    i = i+1;
end

% for fflag=5:1:5
%     T=19+3*(fflag-1);
%     tt = T/(N+2);
%     c = 0;
%     [E,L_E,E_m,E_1,L_E_1,E_m_1,E_F_1,c]=Main_Arig_test(fflag,E,L_E,E_m,E_1,L_E_1,E_m_1,E_F_1,arig,K,N,tt,B_k,B_m,np,P_0,b_0,Q,V,W,H,q_0,T,C);
% end
% end
% for x=2:1:3
%    if x==1
%     W=[-20,60, 0, -40,-75,-50;
%         45,100,170,160,115,50];
%    elseif x==3
%     W=[-20,22,30,40,45,50,60, 75, 30,25, 20, 0, -40,-50,-30;
%         45,50,55,65,70,90,100,115,120,140,160,170,160,50,45];
%     K=15;
%    elseif x==2
%     W=[-20,60,30,25,  20, 0, -40,-75,-50,-30;
%         45,100,120,140,160,170,160,115,50,55];
%    K=10;
%    end

%    arig=3;
%    [E,L_E,E_m,E_1,L_E_1,E_m_1,c]=Main_Arig_test(x,E,L_E,E_m,E_1,L_E_1,E_m_1,arig,K,N,tt,B_k,B_m,np,P_0,b_0,Q,V,W,H,q_0,T,C);
% end

set(gcf,'Units','Inches');
pos = get(gcf,'Position');
set(gcf,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
filename = 'fig17.pdf'; % �趨�����ļ���
print(gcf,filename,'-dpdf','-r0')
close(gcf)

%% �����㷨�������Ż��������7-2.mat��
x=3:1:24
plot(x,E(2,3:24),'o-r',x,E(3,3:24),'s-b',x,E(4,3:24),'d-g',x,E(5,3:24),'p-k','LineWidth',1)
legend('HUECM','FFP','FTOT','MDPP')
axis([3,24,300,1000])
set(gca,'ytick',300:100:1000)
set(gca,'xtick',3:3:24)
xlabel('Number of iterations')
ylabel('Energy consumption(J)')

%% ��ͬʱ�������µĲ�ͬ�㷨�ܺ��Ż�ͼ�������8-1.mat��
x=19:3:31;
plot(x,E_1(2,1:5),'o-r',x,E_1(3,1:5),'s-b',x,E_1(4,1:5),'d-y',x,E_1(5,1:5),'p-k',x,E_1(1,1:5),'x-g','LineWidth',1)
legend('HUECM','FFP','FTOT','MDPP','MC')
axis([19,31,250,1100])
set(gca,'ytick',250:250:1100)
set(gca,'xtick',19:3:31)
xlabel('Time period size(s)')
ylabel('Energy consumption(J)')

x=19:3:31;
plot(x,L_E_1(2,1:5),'o-r',x,L_E_1(3,1:5),'s-b',x,L_E_1(4,1:5),'d-y',x,L_E_1(5,1:5),'p-k',x,L_E_1(1,1:5),'x-g','LineWidth',1)
legend('HUECM','FFP','FTOT','MDPP','MC')
axis([19,31,2,16]);
set(gca,'ytick',2:2:16)
set(gca,'xtick',19:3:31);
xlabel('Time period size(s)')
ylabel('Energy efficiency *1e6(bit/J)');
%% ��ͬ�ն�������µ����������Ż�ͼ�������9-2.mat��
x=[6,10,15];
b=bar(x,E_1.')
grid on
ch=get(b,'children')
legend('HUECM','MC','FFP','FTOT','MDPP')
axis([4,17,0,1200])
set(gca,'ytick',0:200:1200)
set(gca,'xtick',[6,10,15]);
xlabel('Number of terminals')
ylabel('Energy consumption(J)')

x=[6,10,15];
b=bar(x,L_E_1.')
grid on
ch=get(b,'children')
legend('HUECM','MC','FFP','FTOT','MDPP')
axis([4,17,0,18]);
set(gca,'ytick',0:3:18)
set(gca,'xtick',[6,10,15]);
xlabel('Number of terminals')
ylabel('Energy efficiency *1e6(bit/J)');
%% ��ͬʱ϶�����µ������Ż��������10.mat��
x=48:10:68
b=bar(x,E_1.')
grid on
ch=get(b,'children')
legend('HUECM','MC','FFP','FTOT','MDPP')
axis([43,73,0,1000])
set(gca,'ytick',0:100:1000)
set(gca,'xtick',48:10:68);
xlabel('Number of time slot')
ylabel('Energy consumption(J)');

x=48:10:68
b=bar(x,L_E_1.')
grid on
ch=get(b,'children')
legend('HUECM','MC','FFP','FTOT','MDPP')
axis([43,73,0,15])
set(gca,'ytick',0:2:15)
set(gca,'xtick',48:10:68);
xlabel('Number of time slot')
ylabel('Energy efficiency *1e6(bit/J)');
%% ��ͬ�ϴ����������µ������Ż��������11-1.mat��
x=2:2:20
plot(x,E_1(2,1:10),'x-r',x,E_1(3,1:10),'x-b',x,E_1(4,1:10),'x-g',x,E_1(5,1:10),'x-k','LineWidth',2)
legend('HUECM','FFP','FTOT','MDPP')
axis([2,20,300,1000])
set(gca,'ytick',300:100:1000)
set(gca,'xtick',2:2:20)
xlabel('Total amount of offloaded data(bk(1e8)')
ylabel('Energy consumption(J)')
L_E_1=L_E_1./1e6
x=2:2:20
plot(x,L_E_1(2,1:10),'o-r',x,L_E_1(3,1:10),'s-b',x,L_E_1(4,1:10),'d-y',x,L_E_1(5,1:10),'p-k',x,L_E_1(1,1:10),'x-g','LineWidth',1)
legend('HUECM','FFP','FTOT','MDPP','MC')
axis([2,20,1,16])
set(gca,'ytick',1:2:16)
set(gca,'xtick',2:2:20)
xlabel('Total amount of offloaded data *1e8(bit)')
ylabel('Energy efficiency *1e6(bit/J)')