# DC Motor Simulation

This project simulates the behavior of a DC motor using various input parameters. The simulation is visualized using Streamlit, providing interactive sliders for real-time adjustments of the motor parameters and dynamically updating graphs.

## Features

- Interactive sliders for input parameters
- Real-time dynamic graph updates
- Visualizations for motor speed and current over time

## Installation

1. Clone the repository:
    ```sh
    git clone <repository-url>
    cd <repository-folder>
    ```

2. Create a virtual environment (optional but recommended):
    ```sh
    python -m venv venv
    source venv/bin/activate  # On Windows use `venv\Scripts\activate`
    ```

3. Install the required libraries:
    ```sh
    pip install -r requirements.txt
    ```

4. Create a `requirements.txt` file with the following content:
    ```plaintext
    streamlit
    numpy
    matplotlib
    ```

## Usage

1. Run the Streamlit app:
    ```sh
    streamlit run app.py
    ```

2. Open your web browser and go to the provided URL (usually http://localhost:8501).

3. Adjust the sliders in the sidebar to change the DC motor parameters. The graphs will update automatically and dynamically as you adjust the sliders.

## Input Parameters

- **Armature inductance (H)**: Inductance of the motor's armature winding.
- **Armature resistance (Ohms)**: Resistance of the motor's armature winding.
- **Back emf constant (V/(rad/sec))**: Constant representing the back electromotive force.
- **Torque constant (Nm/A)**: Constant representing the torque generated per ampere of current.
- **Moment of Inertia (kg.m^2)**: Moment of inertia of the motor's rotor.
- **Frictional coefficient (Nm/(rad/sec))**: Frictional coefficient of the motor.
- **Time step (s)**: Time step for the simulation.
- **Reference speed (rad/sec)**: Desired reference speed of the motor.
- **Load torque (Nm)**: Load torque applied to the motor.
- **Number of iterations**: Number of iterations for the simulation.

## Code Explanation

The `app.py` file contains the following key components:

- Sidebar sliders for input parameters
- `run_simulation` function to perform the simulation based on input parameters
- Dynamic updating of speed and current graphs using Streamlit's `line_chart`
