## Robot arm controller

This is an app for controlling a robot arm with a dashboard, specifically its angle and the parameters of its PID controller.

https://github.com/user-attachments/assets/cdc2310f-e89a-475c-b49d-2afd1f11a61a



The robot runs in a separate process and it receives and send messages via a channel. The dashboard communicates with the robot via the channel, pushing new parameter updates and periodically retrieving the state of the robot. 

This app showcases:

- Communication between separate entities via channels.
- Using asynchronous tasks for running long processes.
- High-frequency plot updates in real time.

Watch the tutorial [here](https://youtu.be/dsBRj5bstUs?t=1701).

## Installation

Clone the repository and install the dependencies:

First `cd` into the project directory then run:

```bash
$> julia --project -e 'using Pkg; Pkg.instantiate()'
```

Then run the app

```bash
$> julia --project
```

```julia
julia> using GenieFramework
julia> Genie.loadapp() # load app
julia> up() # start server
```

## Usage

Open your browser and navigate to `http://localhost:8000/`
