for i=1:10
    Xi=0.5.*[0;i;0;i];
    Xfi=-1*0.5.*[0;i;0;i];
    Xf2i=-1*0.5.*[0;i;0;-i];
    leader_change_velocity
end