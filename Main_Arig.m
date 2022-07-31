 function [E_m,E,L_E,c]=Main_Arig(i,E_m,E,L_E,arig,K,N,tt,B_k,B_m,np,P_0,b_0,Q,V,W,H,q_0,T,C)
q_E = 0.5;%�����ܺļ�Ȩ��1/2
e1 = 5e-2;%���
E_1_1 = 0;%ͨ���ܺ��Լ������ܺ�
E_F = zeros(5,80);%�㷨1�������ܺ�
E_2=0;
E_1_2 = zeros(5,N);%�㷨1�����ܺ�
E_opt=0;%����ǰһ�ε��ܺ�
E_c = 10;%������������ܺĵ����
L = zeros(1,K);%�û�k����������
L_CL = zeros(K,80);%��ͬʱ�����ڲ�ͬ�û����������Ϣ��
L_CL_m =zeros(K,80);
L_CL_sum = zeros(5,80);
L_CL_m_sum=zeros(5,80);
u = zeros(K,N);%�������ճ���
u_m = zeros(K,N+1);%�������ճ���
y = zeros(1,N+2);%�������ճ���
h_m = zeros(1,N+1);%�Ż����м���
h = zeros(K,N+1);%�Ż����м���

for n=1:1:N
    for k=1:1:K
        u_m(k,n+1) = 0.01;
        u(k,n) =0.02;
    end
    y(n+2) = 1e-4;
end%��ʼ�����ӵ�ֵ
for n = 1:1:N+1
    for k=1:1:K
        h(k,n) = b_0/(pow_pos(norm(Q(:,n)-W(:,k)),2)+H^2);
    end
end%��ʼ��h
for n = 2:1:N+1
    h_m(n) = b_0/(pow_pos(norm(Q(:,n)-q_0(:,1)),2));
end%��ʼ��h_m
c = 1;%��¼���д���
cvx = 1;%��¼��ǰcvx����Ƿ���Ч
pa_0=5e-2;pa_1=5e-3;pa_2=1e-14;
while E_c>e1
    for k=1:1:K%�û�����������
        L(k)=1e10;
    end
    E_opt = E(arig,c);%������һ�ε����ܺ�
    if arig==1||arig==2||arig==4%���������շ������������һ
        [E_1_1,a_m,l_opt,l_m_opt,P_opt,f_u_opt]=Arig1(q_E,K,N,tt,B_k,B_m,np,P_0,h,h_m,u,u_m,y,L,C);
    elseif arig==3%���������շ��������ǹ̶�����
        [E_1_1,a_m,l_opt,l_m_opt,P_opt,f_u_opt]=Arig2(q_E,K,N,tt,B_k,B_m,np,P_0,h,h_m,u,u_m,y,L,C);
    elseif arig==5%�����ԭ��ȷ��a_m��ȡֵ
        [E_1_1,a_m,l_opt,l_m_opt,P_opt,f_u_opt]= Arig3(W,Q,H,q_E,K,N,tt,B_k,B_m,np,P_0,h,h_m,u,u_m,y,L,C);
    end
    if cvx>0&&arig==2%����cvx�㷨
        [E_2,Q,V]=CVX(K,H,T,N,np,b_0,P_0,B_k,B_m,l_opt,l_m_opt,P_opt,W,Q,V);
        E_1_2(arig,:)=E_2(:);
    end
    if arig==1%����CVX�㷨���Ա��㷨һ��
        [E_2,Q,V]=CVX(K,H,T,N,np,b_0,P_0,B_k,B_m,l_opt,l_m_opt,P_opt,W,Q,V);
        E_1_2(arig,:)=E_2(:);
    end
    if cvx>0&&arig==3%һ��cvx�㷨���̶����ʣ��Ա��㷨����
        [E_2,Q,V]=CVX(K,H,T,N,np,b_0,P_0,B_k,B_m,l_opt,l_m_opt,P_opt,W,Q,V);
        E_1_2(arig,:)=E_2(:);
    end
    if cvx>0&&arig==5%һ��cvx�㷨���Ա��㷨�ģ�
        [E_2,Q,V]=CVX(K,H,T,N,np,b_0,P_0,B_k,B_m,l_opt,l_m_opt,P_opt,W,Q,V);
        E_1_2(arig,:)=E_2(:);
    end
    if arig==4%�̶��켣���û��Ͽգ��Ա��㷨����
        [Q,V] = UAV2(N,W);
        [E_2,Q,V]=CVX1(T,N,Q,V);
        E_1_2(arig,:)=E_2(:);
    end
    for n = 1:1:N
        for k=1:1:K
            h(k,n) = h(k,n);
            h(k,n) = b_0/(pow_pos(norm(Q(:,n)-W(:,k)),2)+H^2);
        end
    end%����h
    for n = 1:1:N
        h_m(n+1) = h_m(n+1);
        h_m(n+1) = b_0/pow_pos(norm(Q(:,n+1)-q_0(:,1)),2);
    end%����h_m
    for n = 1:1:N
        for k=1:1:K
            u(k,n) = max(u(k,n)+pa_0*(l_opt(k,n)/(B_k*tt)-a_m(k,n+1)*log2(1+P_0*h(k,n)/np)),0);
            u_m(k,n+1) = max(u_m(k,n+1)+pa_1*(l_m_opt(k,n+1)/(B_m*tt)-a_m(k,n+1)*log2(1+P_opt(k,n+1)*h_m(n+1)/np)),0);
        end
    end%���³���u_m
    l_m = zeros(1,N+1);
    for n = 1:1:N
        for k=1:1:K
            l_m(n+1)=l_m(n+1)+l_m_opt(k,n+1);
        end
        y(n+2) = max(y(n+2)-pa_2*(l_m(n+1)-f_u_opt(n+2)*tt/C),0);
    end%���³���y  
    E_F(arig,c+1) =sum(E_1_2(arig,:));
    if i==1 && abs(E_F(arig,c+1)-E_F(arig,c))<=0.1 %�жϵ�ǰ��cvx�Ż��Ƿ������塣
        cvx=cvx-1;
    end
    E(arig,c+1) =( E_1_1 + E_F(arig,c+1));%���ε�������������ܺ�
    E_m(arig,c+1)=E_1_1*q_E+E_F(arig,c+1);
    E_c = abs(E(arig,c+1)-E_opt);%�������μ���������ܺĵĲ�ֵ
    for k=1:1:K
        for i=1:1:N
            L_CL(k,c+1)= l_opt(k,i)+L_CL(k,c+1);
            L_CL_m(k,c+1) = l_m_opt(k,i)+L_CL_m(k,c+1);
        end
    end  %���㲻ͬ�ܺ��²�ͬ�û����������Ϣ��
    for k=1:1:K
        L_CL_sum(arig,c+1) = L_CL(k,c+1)+L_CL_sum(arig,c+1);
        L_CL_m_sum(arig,c+1) = L_CL_m(k,c+1)+L_CL_m_sum(arig,c+1);
    end  %���㲻ͬ�ܺ������˻��������Ϣ����
    L_E(arig,c+1)=min(L_CL_sum(arig,c+1),L_CL_m_sum(arig,c+1))/E_m(arig,c+1);
    E_c
    
    c=c+1;
end

plot(Q(1,:),Q(2,:),'*-r',W(1,:),W(2,:),'x k',0,100,'o-k');
axis([-80,80,20,180])
xlabel('x(m)')
ylabel('y(m)')
text(45,150,'x����Teminal')
text(45,160,'o����UAV0')
text(45,170,'*����UAV1')

end


