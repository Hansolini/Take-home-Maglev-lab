%% 
teensyRec = serialport("COM6",600000000);
i=0;
while true
    r = read(teensyRec,32,"string");
    if i==0
        tic;
    end
    if i==500000            %Stop after 500000 receives
        toc
        r
        break
    end
    i = i+1;
end
delete(teensyRec)