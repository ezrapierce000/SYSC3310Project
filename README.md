# SYSC3310Project
Final project for SYSC3310

The goal of the project is to have the MSP board change which LEDs are lit based on switch presses and commands sent from the host CPU. As well as to have the changes in LEDs be sent to the Host CPU.

The desired functionality on the MSP board is such:
  -Maintain internal states, corresponding to LEDs
  -Change state when a switch is pressed, forwards or backwards depending on which switch
  -Change state when a message is sent, forward or backwards depending on what message is sent
  -Update host CPU with current state whenever it changes or is requested 
