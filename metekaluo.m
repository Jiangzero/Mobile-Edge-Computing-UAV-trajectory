function [E] = metekaluo(K,N,E_F,tt)
Y_c = 1e-24;
E = inf;
x_max = 10000000;

for x=1:1:x_max
E_1 = 0;    
E_opt = 0;
a = zeros(K,N);
p = zeros(K,N);
f = zeros(1,N);
rng(x);
for n=1:1:N
    a_ = rand(1,K);
    a_max = 0;
    k_max = 0;
    for k=1:1:K        
        if a_(k) > a_max
            a_max = a_(k);
            k_max = k;
        end
    end
    a(k_max,n) = 1;
end%%Ëæ»úa
f = 1.5e8 + (2e8-1.5e8).*rand(1,N);   
p = 0.4 + (0.65-0.4).*rand(K,N); 
for n = 1:1:N
    for k=1:1:K
        E_1=E_1+a(k,n)*p(k,n)*tt;
    end
    E_1=E_1+Y_c*(f(n)^3)*tt;
end
E_opt = E_1+E_F; 
if E_opt < E
    E = E_opt;
end
end

