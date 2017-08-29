# CarND-Path-Planning-Project
When I started the project, I had very well Understood JTM and I was preety sure I was going
to use JTM.So I started with JTM.
My Idea was to get a vehicle Id of vehicle ahead , Get its states and Use Delta to keep car Behind the car or adjacent to car.
So The JTM implementaion was one Herculeus task, Which I put so many hours and got it Ready.
Every Module of JTM like GetStateAt,CostFunction, ... Where coded and tested and verified using python test as refrence.
JTM implementaion was all together a good time to work in c++ after a long time, I enoyed it but then...

Whole module was ready, I plugged in my Idea of working with delta to change lanes.
But my car was not even moving.
So I realized after ouput data on terminal and study of S , that the Value of S range between the car initally was in range of 1000 in magnitude, and I was working like 10's for delta.

In all this experiment I learnt that Maintaing FSM was must. And I need a another method for Startup.



#The Walk thorugh 
Then I choose to got a Distance x, until the SD_vehicle is in range of 30 m of other vehicle then use JTM for lane change.
Implemented the walk though step by step as mentioned, Get Vehicle moving, Decelarate on buffer distance,CHange lane.
So It was clear that with Spline I could do whole project now.
Had hicups initally when spline was braking as I was using Sort after refering some forum discussion.
But while working on spline values I realized the magnitude of Velocity and S distance how it was moving.
These observation were made, to decide the FSM mechnics and all hyper parameters used.


# The Vaccation

I had to spend some time travelling, and thats when I made my FSM design simple as simple and as practical as possible.

# FSM and Design

I implemented a simple state Machine which uses 4 states.
 KL = Keep Lane
 PRP_LC  = Prepare Lane change
 LC = Lane change
 LC_Done = Lane change done

Here we start with Inital state, Speed and lane.
And then keep lane until we have a block ahead.
Then we have to prepare for LC , reduce speed else we will face Max jerk and Max Total acc err.
Total acc err occurs when we change speed in order of 5 m/s and more.
While Max jerk we know occurs when change in accelartaion is of magnitude 5 and above(5 or more...??)
I faced Max total Acc error problem, more that any other issue.

Lane change checks for free left lane and then right.
and finaly when all done we are free to accelarte once Lane chnage done.

This was the most basic approach I used.

It gives me around 5 to 6 miles without incident.

Also The most occuring error is Total acc error.

#What Needs to be done.
Need to work this with JTM implementation. JTM I felt I was so close but didnt take it further for time constarint.
There is no 2 lane chnage happening,At a time only one lane change.If I can make a good two lane change might the efficeny 
will increase.
Over a good project.
This puts me thinking Was JTM mumbo jumbo really needed , when we can solve the same problem in Maths.

# Thanks Udacity!!!
Lastly I thank FOrum mentors and mentors at Udacity for helping me get through the project.
Wothout Walk through this seemed so impossible.
The JTM lesson was well explained as well as the entire unit was well designed.
The Project as well quizes were preety thought provoking.




