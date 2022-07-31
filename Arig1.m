function [E_1,a_m,l_opt,l_m_opt,P_opt,f_u_opt]= Arig1(q_E,K,N,tt,B_k,B_m,np,P_0,h,h_m,u,u_m,y,L,C,P_max)
%P_max = 0.7;%UAV1�����
Y_c = 1e-24;%�����ܺĵĲ���
Q_opt = zeros(K,N+1);%�Ż����м���
a_m = zeros(K,N+1);%�Ż�����������
l_opt = zeros(K,N);%�Ż��������û�k��nʱ϶���͵�������
l_m_opt = zeros(K,N+1);%�Ż����������˻���n+1ʱ϶���͵�������
P_opt = zeros(K,N+1);%�Ż�������UAV1���ʷ���
f_u_opt = zeros(1,N+2);%�Ż�������UAV0����Ƶ��
E_1=0;
for k = 1:1:K
    for n=1:1:N
        if ((u_m(k,n+1)*log(2))/(q_E*tt)-np/h_m(n+1)>P_max)
            P_opt(k,n+1) = P_max;
        elseif (u_m(k,n+1)*log(2))/(q_E*tt)-np/h_m(n+1)<0
            P_opt(k,n+1) = 0;
        else
            P_opt(k,n+1) = ((u_m(k,n+1)*log(2))/(q_E*tt)-np/h_m(n+1));
        end
        Q_opt(k,n+1) =q_E*P_opt(k,n+1)*tt-u_m(k,n+1)*log2(1+ P_opt(k,n+1)*h_m(n+1)/np)-u(k,n)*log2(1+ P_0*h(k,n)/np);
        f_u_opt(n+2) = sqrt(max(y(n+2)/(3*q_E*Y_c*C),0));
    end
end%����P_opt,f_u_opt,Q_opt.
z=1;
for n = 1:1:N
    Q_opt_min=0;
    for k=1:1:K
        if Q_opt(k,n+1)<Q_opt_min
            z = k;
            Q_opt_min = Q_opt(k,n+1);
        end
    end
    if Q_opt(z,n+1)<0
        a_m(z,n+1) = 1;
    else
        a_m(z,n+1) = 0;
    end
end   %����a_m
for k = 1:1:K
    for n=1:1:N
        if  a_m(k,n+1)*log2(1+P_0*h(k,n)/np)*B_k*tt <= L(k)
            l_opt(k,n) = a_m(k,n+1)*log2(1+P_0*h(k,n)/np)*B_k*tt;
            L(k) = L(k)-l_opt(k,n);
        else
            l_opt(k,n) = L(k);
            break;
        end
    end
end%����l_opt
for k=1:1:K%�û�����������
    L(k)=1e9;
end
for k = 1:1:K
    for n=1:1:N
        if  a_m(k,n+1)*log2(1+P_opt(k,n+1)*h_m(n+1)/np)*B_m*tt <= L(k)
            l_m_opt(k,n+1) = a_m(k,n+1)*log2(1+P_opt(k,n+1)*h_m(n+1)/np)*B_m*tt;
            L(k) = L(k)-l_m_opt(k,n+1);
        else
            l_m_opt(k,n+1) = L(k);
break;
        end
    end
end%����l_m_opt
for n = 1:1:N
    for k=1:1:K
        E_1=E_1+a_m(k,n+1)*P_opt(k,n+1)*tt;
    end
    E_1=E_1+Y_c*(f_u_opt(n+2)^3)*tt;
end%ͨ�źͼ�������ܺ�
end


