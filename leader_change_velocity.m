del_t = 0.001;

% for leader
Xli = Xi;
v=[0 1 0 0;0 0 0 0;0 0 0 1;0 0 0 0];
f=[0 0;1 0;0 0;0 1];

% for follower1
tf = 2;
q1=0.2;
p1=2;

% for follower 2
tf = 2;
q2=-0.2;
p2=2;

%PI leader parameters
Kpl=-[1 0; 0 1]*20; Kil=-[1 0; 0 1]*0.1 ;Kdl=-25*[1 0;  0 1];
%PI follower parameters
Kpf=-20; Kif=-0.5;Kdf=-4    ;

Xl_path=[];Ul_path=[];Xf_path=[];Xf2_path=[];Xldes_path=[];Xf_des_path=[];Xf2_des_path=[];Eli=zeros(4,1);Epl=Eli;Efi=zeros(2,1);Epf=Efi;Ef2i=zeros(2,1);Epf2=Ef2i;
% Simulations 
for t=0:del_t:30
    Xdes = [1*cos(t*pi/30-pi/2);-1*pi/30*sin(t*pi/30-pi/2); 1*sin(t*pi/30-pi/2)+1;1*pi/30*cos(t*pi/30-pi/2)];
    Xldes_path = horzcat(Xldes_path,Xdes);
    Xl_path = horzcat(Xl_path,Xli);
    El = Xli-Xdes;
    Eli = Eli+El*del_t;
    Eld = (El-Epl)/del_t;
    if t==0
        Ul = [0;0];
    else
        Ul = Kpl*[El(1); El(3)] + Kil*[Eli(1); Eli(3)] + Kdl*[Eld(1); Eld(3)];
    end
    Ul_path = horzcat(Ul_path, Ul);
    dXl = (v*Xli)+(f*Ul);
    Epl = El;
    
    if t<=tf+del_t
        Xfdes = [Xi(1);Xi(3)];
        Xf2des = [Xi(1);Xi(3)];
    else
        Xl_tf = Xl_path(:,int16((t-tf)/del_t+2));
        Xl_tf_del = (Xl_path(:,int16((t-tf)/del_t+2))-Xl_path(:,int16((t-tf)/del_t)+1))/del_t;
        sin_phi = Xl_tf_del(3)/sqrt(Xl_tf_del(1)^2+Xl_tf_del(3)^2);
        cos_phi = Xl_tf_del(1)/sqrt(Xl_tf_del(1)^2+Xl_tf_del(3)^2);
        Xfdes = [Xl_tf(1)+q1*sin_phi;Xl_tf(3)-q1*cos_phi];
        
        Xl_tf = Xl_path(:,int16((t-tf)/del_t+2));
        Xl_tf_del = (Xl_path(:,int16((t-tf)/del_t+2))-Xl_path(:,int16((t-tf)/del_t)+1))/del_t;
        sin_phi = Xl_tf_del(3)/sqrt(Xl_tf_del(1)^2+Xl_tf_del(3)^2);
        cos_phi = Xl_tf_del(1)/sqrt(Xl_tf_del(1)^2+Xl_tf_del(3)^2);
        Xf2des = [Xl_tf(1)+q2*sin_phi;Xl_tf(3)-q2*cos_phi];
    end
    
    Xf_des_path = horzcat(Xf_des_path,Xfdes);
    Xf_path = horzcat(Xf_path,Xfi);
    Ef = [Xfi(1);Xfi(3)]-Xfdes;
    Efi = Efi+Ef*del_t;
    Edf = (Ef-Epf)/del_t;
   
    if t==0
        Uf = [0;0];
    else
        Uf = Kpf*Ef + Kif*Efi+ Kdf*Edf;
    end
    dXf = (v*Xfi)+(f*Uf);
    Epf=Ef;
    
    Xf2_des_path = horzcat(Xf2_des_path,Xf2des);
    Xf2_path = horzcat(Xf2_path,Xf2i);
    Ef2 = [Xf2i(1);Xf2i(3)]-Xf2des;
    Ef2i = Ef2i+Ef2*del_t;
    Edf2 = (Ef2-Epf2)/del_t;
    if t==0
        Uf2 = [0;0];
    else
        Uf2 = Kpf*Ef2 + Kif*Ef2i+ Kdf*Edf2;
    end
     dXf2 = (v*Xf2i)+(f*Uf2);
     Epf2=Ef2;
    
    Xli = dXl*del_t + Xli;
    Xfi = dXf*del_t + Xfi;
    Xf2i = dXf2*del_t + Xf2i;
end
figure(1)
plot(Xl_path(1,:),Xl_path(3,:),'b',Xldes_path(1,:),Xldes_path(3,:),'k', Xf_path(1,:), Xf_path(3,:),'r',Xf_des_path(1,:),Xf_des_path(2,:),'g',Xf2_path(1,:), Xf2_path(3,:),'m',Xf2_des_path(1,:),Xf2_des_path(2,:),'c');
legend('Leader','Leader desired path', 'Follower', 'Follower desired path','Follower2', 'Follower2 desired path');
hold on
t = 0:del_t:30
;
figure(2)
title('x component of Position')
plot(t,Xl_path(1,:),'r',t,Xf_path(1,:),'b',t,Xf2_path(1,:),'g');
legend('Leader','Follower1','Follower2')
hold on
figure(3)
title('y component of Position')
plot(t,Xl_path(3,:),'r',t,Xf_path(3,:),'b',t,Xf2_path(3,:),'g');
legend('Leader','Follower1','Follower2')
hold on