function [E_2,Q,V]=CVX(K,H,T,N,np,b_0,P_0,B_k,B_m,l_opt,l_m_opt,P_opt,W,q_t,v_r)
tt=T/(N+2);%时隙长度
q_0 = [0;100];%UAV0的初始位置
G = 9.8; %重力加速度
C_1 = 0.002; %飞行能耗参数
C_2 = 70.698; %飞行能耗参数
A_max = 50; %最大加速度5
V_max = 100; %最大速度50m/s
x1=b_0/(np);

cvx_begin
variable Y(N) nonnegative
variables V(2,N+1) A(2,N) Q(2,N+1)
expressions  E_2(N)


for i = 1:1:N
    E_2(i) = (tt*C_1.*pow_pos(norm(V(:,i)),3)+tt*C_2.*inv_pos(Y(i))+ tt*C_2/(G^2).*quad_over_lin((A(:,i)),Y(i)));%目标函数的计算
end

minimize (sum(E_2))
subject to
for i=1:1:N
    for j=1:1:K
        log(1+P_0*x1*inv_pos(pow_pos(norm(q_t(:,i)-W(:,j)),2)+pow_pos(H,2)))-P_0*x1*(pow_pos(norm(Q(:,i)-W(:,j)),2)-pow_pos(norm(q_t(:,i)-W(:,j)),2))*inv_pos(pow_pos(norm(q_t(:,i)-W(:,j)),2)+pow_pos(H,2))*inv_pos(pow_pos(norm(q_t(:,i)-W(:,j)),2)+pow_pos(H,2)+P_0*x1) >= log(2)*inv_pos(B_k*tt)*l_opt(j,i) ;%C1%用户上传数据到UAV1的数据量约束
        log(1+P_opt(j,i+1)*x1*inv_pos(pow_pos(norm(q_t(:,i+1)-q_0(:,1)),2)))-P_opt(j,i+1)*x1*(pow_pos(norm(Q(:,i+1)-q_0(:,1)),2)-pow_pos(norm(q_t(:,i+1)-q_0(:,1)),2))*inv_pos(pow_pos(norm(q_t(:,i+1)-q_0(:,1)),2))*inv_pos(pow_pos(norm(q_t(:,i+1)-q_0(:,1)),2)+P_opt(j,i+1)*x1)  >= log(2)*inv_pos(B_m*tt)*l_m_opt(j,i+1);%C3
    end
end
Q(:,1) == Q(:,N+1)%C12
V(:,1) == V(:,N+1)%C13

for i=1:1:N
    Q(:,i+1) - Q(:,i) == V(:,i).*tt+0.5.*A(:,i).*tt.*tt %下一个时隙位置的约束%C10
    V(:,i+1) - V(:,i) == A(:,i).*tt %下一个时隙速度约束%C11
    power(Y(i),2) <= pow_pos(norm(v_r(:,i)),2)+2*v_r(:,i)'*(V(:,i)-v_r(:,i))
    norm(Q(:,i+1)-Q(:,i)) <= V_max*tt%C16
    norm(A(:,i)) <= A_max%C15
end
for i=1:1:N+1
    norm(V(:,i)) <= V_max%C14
end
Y >= 0
cvx_end

