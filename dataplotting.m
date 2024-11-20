clc;
close all;

data = readtable("~/Documents/firmware/100ms-100pw.txt");
angle = data.Var1;
angle = unwrap(angle);
time = 1:length(angle);

plot(time, angle);
