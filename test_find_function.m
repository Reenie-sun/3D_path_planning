clear;
clc;


test = magic(10);

test2 = [ 1 2 3 4 5 6 7 8 9; 1 2 9 4 5 6 7 8 9; 1 2 3 4 6 6 7 8 9; 1 2 3 4 8 6 7 8 9; 1 2 3 4 5 6 7 8 9 ];

test3 = [ 5 2 1 ];

a = [ 1 3 5];

%result = find( test3 == a )

for i = 1 : length(a)
   
    result = find( a(i) == test3 );
    
    result
    
   test3(result)
    
end