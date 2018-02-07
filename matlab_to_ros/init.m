clear all
clc

duration = 10;
Ts = 0.01;
time = (0:Ts:duration);
samples = numel(time);

hand=zeros(samples,1).';
velvet=zeros(samples,1).';
q_right=zeros(samples,7).';
q_left=zeros(samples,7).';

q_right_timeser = timeseries(q_right, time);
q_left_timeser = timeseries(q_left, time);
tool_hand = timeseries(hand, time);
tool_velvet = timeseries(velvet, time);