

ptlv = [0 0];
piov = [0 0];

%ss = SockStaubli('localhost',1111,3);
%ss.connect();
for i = [-0.1:0.001:0.1]
    ptlv = [0 i ];
    piov=[0 0];
    cmd = [ piov 0 0 ptlv ];
    ss.send(cmd);
    pause(0.02);
    [vels,here,empty,ok] = ss.rcv_noblk();
    m1=sprintf('cmd %7.2f odo %7.2f %7.2f %7.2f theta %7.2f ptl %7.2f %7.2f',cmd(6),vels(1),vels(2),vels(3),vels(4),vels(5),vels(6));
    m2=sprintf('x %7.2f y %7.2f z %7.2f',here(4),here(5),here(6));
    display([m1 m2]);

end
%ss.close();