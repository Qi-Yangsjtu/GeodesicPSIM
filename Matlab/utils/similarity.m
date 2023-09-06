function sim = similarity(a,b)
t=eps;
sim = abs(2.*a.*b+t)./(a.^2+b.^2+t);
