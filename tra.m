function  [Q,V]= tra(N,tt)
n=N+1;
Q=zeros(2,n);
V=zeros(2,n-1);
x=zeros(1,N+1);
    u=360/N;
    for i=1:1:N
        Q(1,i+1)=100*cosd(-90+u*i);
        Q(2,i+1)=100+100*sind(-90+u*i);
    end
    for i=1:1:N
        V(1,i+1)=2*(Q(1,i+1)-Q(1,i))/tt-V(1,i);
        V(2,i+1)=2*(Q(2,i+1)-Q(2,i))/tt-V(2,i);
    end   
end
  
