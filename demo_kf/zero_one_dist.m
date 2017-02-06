function noise = zero_one_dist(p)
   noption = [0.8,0,0.4];
   noise = randi(100,1,5);
   for ki=1:5
           if noise(ki) <= p*100/2
			    noise(ki) = noption(1);
		   else if noise(ki) >= (100 - p*100/2)
				          noise(ki) = noption(3);
			   else
				           noise(ki)= noption(2);
			       end
		   end	   
   end
end