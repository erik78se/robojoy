# Adding encoders

Right, this week, [Jimmy] was out bycyling in Italy so this week was even more slackier than last. 
I did however have some time to watch a pretty good informative video about "Encoders": https://www.youtube.com/watch?v=oLBYHbLO8W0

Adding in our encoders to the hardware, is on the next "todo" in our project, so that made alot of sense to me to watch.

# Thoughts about 4 motors
When thinking about encoders, its a thought that came up in my head, that we have 4 motors where we only attach encoders to 2 of them. 
Splitting them up into 4 different controllers wont work, since we only have 2 outputs for M1, M2 on our A*^motor controller. 
I suspect we will end up in some sort of sync problems with the motors. Lesson learned here sync #motors with #controllers?

# Arduino blowup
[Erik] managed to burn his arduino while playing with 240V relays. He is bying a new one for testing purposes.

# This week milestone (M3)
Check out the items from [Milestone 3](https://github.com/erik78se/robojoy/milestone/3)


[![Build Status](https://travis-ci.com/erik78se/robojoy.svg?branch=master)](https://travis-ci.com/erik78se/robojoy.svg?branch=master)

   [Jimmy]: <https://github.com/HeMan>
   [Erik]: <https://github.com/erik78se>


