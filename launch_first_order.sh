source devel/setup.bash

rb1x=$(python -c 'import random;print (random.random()-0.5)*10')
rb2x=$(python -c 'import random;print (random.random()-0.5)*10')
rb3x=$(python -c 'import random;print (random.random()-0.5)*10')
rb1y=$(python -c 'import random;print (random.random()-0.5)*10')
rb2y=$(python -c 'import random;print (random.random()-0.5)*10')
rb3y=$(python -c 'import random;print (random.random()-0.5)*10')

roslaunch launcher launcher-first_order.launch rb1x:=$rb1x rb2x:=$rb2x rb3x:=$rb3x rb1y:=$rb1y rb2y:=$rb2y rb3y:=$rb3y 
