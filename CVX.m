function [E_2,Q,V]=CVX(K,H,T,N,np,b_0,P_0,B_k,B_m,l_opt,l_m_opt,P_opt,W,q_t,v_r)
tt=T/(N+2);%ʱ϶����
q_0 = [0;100];%UAV0�ĳ�ʼλ��
G = 9.8; %�������ٶ�
C_1 = 0.002; %�����ܺĲ���
C_2 = 70.698; %�����ܺĲ���
A_max = 50; %�����ٶ�5
V_max = 100; %����ٶ�50m/s
x1=b_0/(np);

cvx_begin
variable Y(N) nonnegative
variables V(2,N+1) A(2,N) Q(2,N+1)
expressions  E_2(N)


for i = 1:1:N
    E_2(i) = (tt*C_1.*pow_pos(norm(V(:,i)),3)+tt*C_2.*inv_pos(Y(i))+ tt*C_2/(G^2).*quad_over_lin((A(:,i)),Y(i)));%Ŀ�꺯���ļ���
end

minimize (sum(E_2))
subject to
for i=1:1:N
    for j=1:1:K
        log(1+P_0*x1*inv_pos(pow_pos(norm(q_t(:,i)-W(:,j)),2)+pow_pos(H,2)))-P_0*x1*(pow_pos(norm(Q(:,i)-W(:,j)),2)-pow_pos(norm(q_t(:,i)-W(:,j)),2))*inv_pos(pow_pos(norm(q_t(:,i)-W(:,j)),2)+pow_pos(H,2))*inv_pos(pow_pos(norm(q_t(:,i)-W(:,j)),2)+pow_pos(H,2)+P_0*x1) >= log(2)*inv_pos(B_k*tt)*l_opt(j,i) ;%C1%�û��ϴ����ݵ�UAV1��������Լ��
        log(1+P_opt(j,i+1)*x1*inv_pos(pow_pos(norm(q_t(:,i+1)-q_0(:,1)),2)))-P_opt(j,i+1)*x1*(pow_pos(norm(Q(:,i+1)-q_0(:,1)),2)-pow_pos(norm(q_t(:,i+1)-q_0(:,1)),2))*inv_pos(pow_pos(norm(q_t(:,i+1)-q_0(:,1)),2))*inv_pos(pow_pos(norm(q_t(:,i+1)-q_0(:,1)),2)+P_opt(j,i+1)*x1)  >= log(2)*inv_pos(B_m*tt)*l_m_opt(j,i+1);%C3
    end
end
Q(:,1) == Q(:,N+1)%C12
V(:,1) == V(:,N+1)%C13

for i=1:1:N
    Q(:,i+1) - Q(:,i) == V(:,i).*tt+0.5.*A(:,i).*tt.*tt %��һ��ʱ϶λ�õ�Լ��%C10
    V(:,i+1) - V(:,i) == A(:,i).*tt %��һ��ʱ϶�ٶ�Լ��%C11
    power(Y(i),2) <= pow_pos(norm(v_r(:,i)),2)+2*v_r(:,i)'*(V(:,i)-v_r(:,i))
    norm(Q(:,i+1)-Q(:,i)) <= V_max*tt%C16
    norm(A(:,i)) <= A_max%C15
end
for i=1:1:N+1
    norm(V(:,i)) <= V_max%C14
end
Y >= 0
cvx_end

