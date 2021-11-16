if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end
device1 = serial('COM4','BaudRate',9600);
fopen(device1)
fwrite(device1,'0','char')
fwrite(device1,'2','char')
fwrite(device1,'0','char')
fwrite(device1,'1','char')
fwrite(device1,'0','char')
fclose(device1)
