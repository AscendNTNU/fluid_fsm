Simen Gangstad 16/06/19: This document states the initial thoughts of the FSM, it is not meant as a an accurate reference. 



## How to read this document
References to sources are given with e.g. (1, 2). Where the first number in the tuple is a reference to the source
and the second number is a reference to the given page/slide/time of a video.


## Generic implementation
A finite state machine is constructed of the following components (1, 5):
- Set of states
- Set of symbols/arguments which serves as possible inputs
- Transition function that performs a state change based on input

Mathematically one can define a finite state machine with the tuple (E, T, S, S0, D) (1, 9). Where:
- E is the input symbols/arguments
- T is the output
- S is a finite, non-empty set of states
- S0 is the initial state
- D is the transition function D : S x E -> S x T


### Transition
The mathematical model for the transition function is defined by taking in an input state (S) and an argument (E), and
returning an output state (S) and a response (T). Therefore, in order to transition, a request should be sent to the
FSM. The FSM can either deny this request because the state transition doesn't make sense, or of course carry it out.
Conversely, the response would encapsulate a success flag and possibly some output data.

[fsm diagram]: https://upload.wikimedia.org/wikipedia/commons/thumb/7/71/Fsm_Moore_model_door_control.svg/512px-Fsm_Moore_model_door_control.svg.png

As one can see from the illustration (2), a request should include an argument, e.g. "close" or "open", and the current
state. The FSM will determine based on the argument and the current state whether that transition is valid or not.


### TL;DR
In short, what the finite state machine requires:
- States
- A transition function which determines if a state change is valid based on input. The output from the transition
  function is a response with a completion flag and possibly some data.


## Technical implementation

### States
Building on ROS and Ardupilot, a state should encapsulate:
- a pose
- an action
- callbacks for when a state begins execution and when a state is finished executing

#### Different types of states

- Manual (Ardupilot: MANUAL/STABILIZED)
- Idle (armed, but stationary at ground)
- Take off (Ardupilot: MIS_TAKEOFF_ALT)
- Hold (hovers at current position, Ardupilot: AUTO_LOITER)
- Move (Ardupilot: GUIDED)
- Land (lands at the current position, Ardupilot: AUTO_RTL)
- Kill (death switch for the judges)

### Transition
A transition is just a state change, so the essentials are:
- Start state
- End state
- Completion callback with errors (if any)
- Communication with the FSM on Ardupilot


### Operation
The general object of the finite state machine is to carry out a series of transitions in order to achieve some sort of
goal. Further in this document this goal will be called an **operation**. An operation consists of a series of
transitions based on the current state. E.g. if the drone is stationary at the ground, and someone issues an operation
to move to another point in space which also is stationary at the ground. The operation would have to consist of the
following five states and four transitions:
1. Idle
- Transition to elevate state
2. Elevate
- Transition to move state
3. Move to x, y + elevated constant, z
- Transition to land state
4. Land at x, y, z
- Transition to idle state
5. Idle

The FSM in its core will only handle state changes, but Fluid will consist of an extra layer outside this
which implements a state graph for operations.


##### Actionlib

Since the operations might take arbitrary amount of time, an asynchronous system for sending/receiving data is
necessary. ROS actionlib (4) is sufficient for this. Fluid should therefore implement an **ActionServer**. The clients,
which in our example are the operation callers, have to implement **ActionClient**.

The action specification consists of:
- Goal (the data of the operation. E.g. for a move operation this would be the set point)
- Feedback (progress feedback from the server)
- Result (the output of the operation, e.g. the final position)

Action files are placed within the ./action folder as example.action. An example of an action file:
```
# Define the goal
uint32 dishwasher_id  # Specify which dishwasher we want to use
---
# Define the result
uint32 total_dishes_cleaned
---
# Define a feedback message
float32 percent_complete
```

It is necessary to implement the following in CMakeList:
```
find_package(catkin REQUIRED genmsg actionlib_msgs actionlib)
add_action_files(DIRECTORY action FILES DoDishes.action)
generate_messages(DEPENDENCIES actionlib_msgs)
```


And this in the package.xml:
 ```
 <build_depend>actionlib</build_depend>
 <build_depend>actionlib_msgs</build_depend>
 <exec_depend>actionlib</exec_depend>
 <exec_depend>actionlib_msgs</exec_depend>
 ```

Then the .msg files are generated with genaction.py.


### State Graph
A transition is just a state change. In order to make this as flexible as possible it's natural to implement some sort
of graph for the different states and run through this and check if a certain transition is allowed or not.

It's important to notice in this design the transitions don't care if the state change is valid or not. If Ardupilot
complains, the transition completion handler will return an error of course, but **the state graph is responsible
for figuring out which transitions that are valid**. Therefore will transitions most likely be a private API
for Fluid, while operations are public.


### TL;DR
Fluid will have operations which you can call (using action lib) and these operations encapsulates states and transitions. The transitions manage flight mode changes in Ardupilot.


## Communication with the FSM in Ardupilot
Transitions will take care of switching between different flight modes in Ardupilot.

### Error handling
If an error occurs it's the transition's job to pass this error further to the completion handler. In Fluid this will
end up at the operation. This should not happen of course, as the state graph should be designed in such a way that
every valid transition in the graph is also valid in Ardupilot. But if an error occurs, there should be a set of methods for
dealing with them. E.g.:
- Wait and try after a short interval

### Sources
1. Marco Della Vedova, *Robotics Finite State Machines* - http://robot.unipv.it/toolleeo/teaching/docs_robotics/fsm.pdf
2. *Finite-state machine* - https://en.wikipedia.org/wiki/Finite-state_machine
3. *Finite State Machine (Finite Automata)* - https://www.youtube.com/watch?v=Qa6csfkK7_I
4. *Action lib* - http://wiki.ros.org/actionlib
5. *Flight modes* - https://dev.px4.io/en/concept/flight_modes.html
