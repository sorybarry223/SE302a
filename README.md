# My project
My project has an issue. The values computed by the output of the gyroscope are too high I think even multiplied by the full scale. I had to divide them by 10 in order to get usable values. Thats a problem I could not solve. 
The second issue occured in writing the function void dm163_turn_off_row(const struct device *dev, const struct gpio_dt_spec
row);
In fact I could not use it this way I used the following prototype:
void dm163_turn_off_row(const struct device *dev, const struct gpio_dt_spec *rows,int row);

It looks like it works, but even if I dont use it, the code still seems to work so I am a little skeptic about it. I cant know whether this function works or not.

I neither could find a way to include functions and threads from the tilt angles project into the matrix project so I just reused almost everything in the tilt angles project which make my code very difficult to read.
