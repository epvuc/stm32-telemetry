** Switching the system clock from HSI to MSI @ 2mhz drops the baseline
current draw while awake from 6.4mA to 1.1mA. 

Best case - received on first try, single blink. 
66ms @ 1.1mA    72.6
1ms @ 20mA      20
10ms @ 1.5mA    15
29923ms @ .0065 195

        total 302.6

302.6 uA*sec  * 2880 sec/day = .242 mAH/day
700 mAH cell -> 7.9 years


Worst case - no receiver, so max retries and double blink at the end. 

2.6 mA average over 259 mS = 673 uA*sec
.0064 mA over 29741 mS = 190 uA*sec
863 uA*sec  -- > .000239 mAH * 2800 cycles/day --> .69 mAH/day
700 mAH cell -> 2.7 years


----------------
below here are measurements with it using HSI clock instead of MSI


66 ms @ .14v 		6.4mA
1.7ms per attempt: 
    .7ms @ 0.420v	19mA
    1 ms @ 0.136v	6.1mA

10 ms @ .232v		10.5mA
100ms @ .136v		6.1mA
10 ms @ .232v		10.5mA


66ms @ 6.4mA  = 422.4 uA * S
15 retries    = 310.4
blink1	      = 105
100ms dly     = 610
blink2        = 105
29.8 s @ 6.5uA= 194

total: 1746 uA * S = 0.000485 mA*hr per 30 sec cycle. 
1.746 mA * sec
.000485 mA * hr

.000485 mA*hr * 2880 cycles / day =  1.4 mA*hr /day (worst case)
700 mAH cell --> 500 days = 1.3 yrs

best case: 
66ms @ 6.4mA		422
1ms @ 20mA		 20
10ms @ 10.5mA		105
29923 ms @ 0.0065 mA	195
total 742 uA*sec / 30 sec
.000206 mA*hr * 2880 = .593 mA*hr / day
700 mAH cell --> 1180 days = 3.2 yrs


normal measured cycle with 3 retries
.13/21.9
.0059360730
.45 / 21.9
.0205479452
.24 / 21.9
.0109589041
.24 / 21.9
.0109589041

(.4 * 21) + (.1 * 11) + (.25 * 21) + (1* 6) 
20.75
(.4 * 21) + (.1 * 11) + (.25 * 21) + (.6* 6) 
18.35
66 * 6
396

396 + 20.75 + 20.75 + 18.35 + 110
565.85
566 / 1000/3600
.0001572222 mA*hr / cycle * 2880 = 
.453 mA*hr / day

700 mAH cell --> 1545 days = 4.2 years
