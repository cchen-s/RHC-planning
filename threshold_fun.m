sigma_x=13;
sigma_y=30;
R=5;

threshold=0.8;
a=sqrt(-2*log((1-threshold)));

p = 1- exp(-0.5*(a*sigma_x+R)^2/sigma_x^2)



