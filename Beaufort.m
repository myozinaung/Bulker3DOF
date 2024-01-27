function [U_wind, Hs] = Beaufort(BF_No)
if BF_No == 0
    U_wind  = 0;
    Hs      = 0;
    
elseif BF_No == 1
    U_wind  = 0.9;
    Hs      = 0.1;

elseif BF_No == 2
    U_wind  = 2.45;
    Hs      = 0.2;  
    
elseif BF_No == 3
    U_wind  = 4.4;
    Hs      = 0.6;   
    
elseif BF_No == 4
    U_wind  = 6.7;
    Hs      = 1;
    
elseif BF_No == 5
    U_wind  = 9.35;
    Hs      = 2; 
    
elseif BF_No == 6
    U_wind  = 12.35;
    Hs      = 3;
    
elseif BF_No == 7
    U_wind  = 15.55;
    Hs      = 4;
    
elseif BF_No == 8
    U_wind  = 19;
    Hs      = 5.5;    
    
elseif BF_No == 9
    U_wind  = 22.65;
    Hs      = 7;
    
elseif BF_No == 9.5
    U_wind  = 24.6;
    Hs      = 8;
    
elseif BF_No == 10
    U_wind  = 26.5;
    Hs      = 9;
elseif BF_No == 11
    U_wind  = 30.55;
    Hs      = 11.5;
else
    U_wind  = 0.1;
    Hs      = 0;
    
end