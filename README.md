## Robot arm controller

This is an app for controlling a robot arm with a dashboard, specifically its angle and the parameters of its PID controller.


https://github.com/user-attachments/assets/bc429e52-0919-4209-9372-d03e3d9030ca


The robot runs in a separate process and it receives and send messages via a channel. The dashboard communicates with the robot via the channel, pushing new parameter updates and periodically retrieving the state of the robot. 

This app showcases:

- Communication between separate entities via channels.
- Using asynchronous tasks for running long processes.
- High-frequency plot updates in real time.

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
