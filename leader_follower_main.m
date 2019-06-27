del_t = 0.001;
% for leader 
Xi=[0;0];
Xli = Xi;
% for follower1
Xfi=[0;0];
tf = 2;
q1=0.2;
p1=2;
% for follower 2
Xf2i=[0;0];
tf = 2;
q2=-0.2;
p2=2;

%PI leader parameters
Kpl=-[1 0; 0 1] ; Kil=-[0 0; 0 0] ;
%PI follower parameters
Kpf=-1; Kif=0.00;

Xl_path=[];Ul_path=[];Xf_path=[];Xf2_path=[];Xldes_path=[];Xf_des_path=[];Xf2_des_path=[];Eli=zeros(2,1);Efi=zeros(2,1);Ef2i=zeros(2,1);
% Simulations 
for t=0:del_t:30
    Xdes = [1*cos(t*pi/30-pi/2); 1*sin(t*pi/30-pi/2)+1];
    Xldes_path = horzcat(Xldes_path,Xdes);
    Xl_path = horzcat(Xl_path,Xli);
    El = Xli-Xdes;
    Eli = Eli+El;
    if t==0
        Ul = [0;0];
    else
        Ul = Kpl*El(1:2) + Kil*Eli(1:2)/t;
    end
    Ul_path = horzcat(Ul_path, Ul);
    dXl = Ul;
    
    if t<=tf
        Xfdes = Xi;
        Xf2des = Xi;
    else
        Xl_tf = Xl_path(:,int16((t-tf)/del_t+2));
        Xl_tf_del = (Xl_path(:,int16((t-tf)/del_t+2))-Xl_path(:,int16((t-tf)/del_t)+1))/del_t;
        sin_phi = Xl_tf_del(2)/sqrt(Xl_tf_del(1)^2+Xl_tf_del(2)^2);
        cos_phi = Xl_tf_del(1)/sqrt(Xl_tf_del(1)^2+Xl_tf_del(2)^2);
        Xfdes = [Xl_tf(1)+q1*sin_phi; Xl_tf(2)-q1*cos_phi];
        
        Xl_tf = Xl_path(:,int16((t-tf)/del_t+2));
        Xl_tf_del = (Xl_path(:,int16((t-tf)/del_t+2))-Xl_path(:,int16((t-tf)/del_t)+1))/del_t;
        sin_phi = Xl_tf_del(2)/sqrt(Xl_tf_del(1)^2+Xl_tf_del(2)^2);
        cos_phi = Xl_tf_del(1)/sqrt(Xl_tf_del(1)^2+Xl_tf_del(2)^2);
        Xf2des = [Xl_tf(1)+q2*sin_phi; Xl_tf(2)-q2*cos_phi];
    end
    
    Xf_des_path = horzcat(Xf_des_path,Xfdes);
    Xf_path = horzcat(Xf_path,Xfi);
    Ef = Xfi-Xfdes;
    Efi = Efi+Ef;
    if t==0
        Uf = [0;0];
    else
        Uf = Kpf*Ef(1:2) + Kif*Efi(1:2)/t;
    end
    dXf = Uf;
    
    Xf2_des_path = horzcat(Xf2_des_path,Xf2des);
    Xf2_path = horzcat(Xf2_path,Xf2i);
    Ef2 = Xf2i-Xf2des;
    Ef2i = Ef2i+Ef2;
    if t==0
        Uf2 = [0;0];
    else
        Uf2 = Kpf*Ef2(1:2) + Kif*Ef2i(1:2)/t;
    end
    dXf2 = Uf2;
    
    Xli = dXl*del_t + Xli;
    Xfi = dXf*del_t + Xfi;
    Xf2i = dXf2*del_t + Xf2i;
end
figure(1)
plot(Xl_path(1,:),Xl_path(2,:),'b',Xldes_path(1,:),Xldes_path(2,:),'k', Xf_path(1,:), Xf_path(2,:),'r',Xf_des_path(1,:),Xf_des_path(2,:),'g',Xf2_path(1,:), Xf2_path(2,:),Xf2_des_path(1,:),Xf2_des_path(2,:));
legend('Leader','Leader desired path', 'Follower', 'Follower desired path','Follower2', 'Follower2 desired path');