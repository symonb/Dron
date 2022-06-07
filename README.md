# Drone 
My own control system - written from scratch. The main reason to do this was my fascination with FPV drones and my wish to deeply understood what is going on behind the scene. Also, I wanted to learn to program and since I like tough challenges, this seemed to be a good choice. I try to do everything on registers and use as least external libraries as possible. For sure, it is far from an optimal code but allows for a deep understanding of low-level, embedded-system programming. Moreover, it is more readable (I hope) than "professional" generally available software. I hope it can be useful for someone to understand some of the features and capabilities of more advanced programs.
## FLY MODES
For now there are 2 options Acro and Stabilize. For all familiar with FPV drones it should be quite clear but for all who wants to know more encourage to visit my blog where it is explained in details. [link](https://symonbielenin.blogspot.com/).
### ACRO


### STABILIZE
Made on quaternions 

## FILTERING DATA
Simple implementation of FIR and IRR filters allow for some tweaks and plays with data live processing. 

## BLACKBOX
Allow for collecting data from flight and next transfer to the computer via UART based radiotrasmitter. 


