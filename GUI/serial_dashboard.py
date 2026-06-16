import serial
import threading
import collections
import pandas as pd
from dash import Dash, dcc, html
from dash.dependencies import Input, Output, ALL, State
from dash.exceptions import PreventUpdate
import plotly.graph_objs as go
from plotly.subplots import make_subplots
import logging


SERIAL_PORT = '/dev/cu.usbmodem101'
SERIAL_PORT_2 = '/dev/cu.usbmodem1101'
BAUD_RATE = 115200
MAX_POINTS = 300
UPDATE_SPEED_MS = 500


# columns of the csv from the serial monitor
COLUMNS = [
    "t_ms","hand_temp_c","hand_rh","wrist_temp_c","wrist_rh","socket_temp_c",
    "socket_rh","bus0_ds1_c","bus0_ds2_c","bus0_ds3_c","bus1_ds1_c",
    "bus1_ds2_c","bus1_ds3_c","socket","battery","direct_ds1_c",
    "imu_ax_g","imu_ay_g","imu_az_g","imu_mag_g","current_ma","bus_mv",
    "power_mw","temp_limit_flag","humidity_limit_flag","overcurrent_limit_flag",
    "shock_flag","power_cut_flag","sensors_ok"
]


# names for printing the same CSV columns
NAMES = {
    "t_ms": "Time [s]",
    "hand_temp_c": "Hand Ambient Temp [°C]",
    "hand_rh": "Hand RH (%)",
    "wrist_temp_c": "Wrist Ambient Temp [°C]",
    "wrist_rh": "Wrist RH (%)",
    "socket_temp_c": "Socket Ambient Temp [°C]",
    "socket_rh": "Socket RH (%)",
    "bus0_ds1_c": "Motor 1 Hand, Bus 1 Temp [°C]",
    "bus0_ds2_c": "Motor 2 Hand, Bus 1 Temp [°C]",
    "bus0_ds3_c": "Motor 3 Hand, Bus 1 Temp [°C]",
    "bus1_ds1_c": "Motor 1 Hand, Bus 2 Temp [°C]",
    "bus1_ds2_c": "Motor 2 Hand, Bus 2 Temp [°C]",
    "bus1_ds3_c": "Motor 3 Hand, Bus 2 Temp [°C]",
    "direct_ds1_c": "Motor 1 Wrist, Direct Temp [°C]",
    "socket": "Socket Contact Temp [°C]",
    "battery": "Battery Temp [°C]",
    "imu_ax_g": "IMU Accel X [g]",
    "imu_ay_g": "IMU Accel Y [g]",
    "imu_az_g": "IMU Accel Z [g]",
    "imu_mag_g": "IMU Mag [g]",
    "current_ma": "Current [mA]",
    "bus_mv": "Voltage (V)",
    "power_mw": "Power [mW]",
}


# plot groups
GROUPS = {
    "Motor Temperatures (°C)": ["bus0_ds1_c","bus0_ds2_c","bus0_ds3_c","bus1_ds1_c","bus1_ds2_c","bus1_ds3_c","direct_ds1_c","socket","battery"],
    "Body Temperatures (°C)": ["hand_temp_c","wrist_temp_c","socket_temp_c"],
    "Relative Humidity (%)": ["hand_rh","wrist_rh","socket_rh"],
    "IMU Data (g)": ["imu_ax_g","imu_ay_g","imu_az_g","imu_mag_g"],
    "Power Data": ["current_ma","bus_mv","power_mw"]
}


# flags for when the interrupt was triggered, to be plotted
FLAGS = [
    "temp_limit_flag","humidity_limit_flag","overcurrent_limit_flag",
    "shock_flag","power_cut_flag","sensors_ok"
]

# Friendly display names for flags
FLAG_NAMES = {
    "temp_limit_flag": "Temperature Limit",
    "humidity_limit_flag": "Humidity Limit",
    "overcurrent_limit_flag": "Overcurrent Limit",
    "shock_flag": "Shock Limit",
    "power_cut_flag": "Power Cut",
    "sensors_ok": "Sensors OK"
}


data_queue = collections.deque(maxlen=MAX_POINTS)
initial_health_check_done = False  # Flag to only check sensor health once at startup
def is_sensor_healthy(value, sensor_type):
    """
    Check if a sensor value is valid based on sensor type.
    Returns True if healthy, False if broken/invalid.
    """
    # Check for None or NaN
    if value is None or (isinstance(value, float) and pd.isna(value)):
        return False


    if sensor_type == "humidity":
        # Humidity should not be 0.0 (sensor issue)
        return value != 0.0
    elif sensor_type == "temperature":
        # Temperature should be >= -10°C (reasonable minimum)
        return value >= -10.0
    elif sensor_type == "imu":
        # IMU readings can be any non-zero value, just check for NaN/None
        return True
    elif sensor_type == "power":
        # Power readings should be positive if something is connected
        return abs(value) > 0
    elif sensor_type == "current":
        # Current readings should be non-negative
        return abs(value) >= 3.0 # resultion of ~1.2 mA, so it sometimes reads small values without anything conneected to it
    elif sensor_type == "voltage":
        # Voltage readings should be non-zero if powered
        return value != 0.0


    return True


def check_sensor_status(df):
    """
    Check which sensors are healthy based on latest data.
    Returns a dict mapping variable names to their health status.
    """
    if df.empty:
        return {}


    latest = df.iloc[-1]
    status = {}


    # Temperature sensors (Body)
    for temp_col in ["hand_temp_c", "wrist_temp_c", "socket_temp_c"]:
        status[temp_col] = is_sensor_healthy(latest[temp_col], "temperature")


    # Humidity sensors
    for rh_col in ["hand_rh", "wrist_rh", "socket_rh"]:
        status[rh_col] = is_sensor_healthy(latest[rh_col], "humidity")


    # Motor temperature sensors
    for motor_col in ["bus0_ds1_c", "bus0_ds2_c", "bus0_ds3_c", "bus1_ds1_c", "bus1_ds2_c", "bus1_ds3_c", "direct_ds1_c", "socket", "battery"]:
        status[motor_col] = is_sensor_healthy(latest[motor_col], "temperature")


    # IMU sensors
    for imu_col in ["imu_ax_g", "imu_ay_g", "imu_az_g", "imu_mag_g"]:
        status[imu_col] = is_sensor_healthy(latest[imu_col], "imu")


    # Power, current and voltage sensors
    status["current_ma"] = is_sensor_healthy(latest["current_ma"], "current")
    status["bus_mv"] = is_sensor_healthy(latest["bus_mv"], "voltage")
    status["power_mw"] = is_sensor_healthy(latest["power_mw"], "power")


    return status


# read serial data and add to queue in seperate thread, not to block the dashboard updates
def read_serial_data():
    import time


    while True:
        try:
            try:
                ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
                print(f"Connected to {SERIAL_PORT}")
            except:
                ser = serial.Serial(SERIAL_PORT_2, BAUD_RATE)
                print(f"Connected to {SERIAL_PORT_2}")
            data_queue.clear()  # Clear old data on reconnection
            print("Data queue cleared - ready for new data")


            while True:
                line = ser.readline().decode('utf-8').strip()


                # Skip setup messages and empty lines
                if line.startswith('#') or not line:
                    continue


                vals = line.split(',')
                if len(vals) == len(COLUMNS) and vals[0] != "t_ms":
                    try:
                        parsed = [float(v) if v.lower() != 'nan' else None for v in vals]
                        data_queue.append(parsed)
                        print(data_queue[-1])
                    except ValueError:
                        pass
        except serial.SerialException as e:
            print(f"Serial connection error: {e}")
            print("Attempting to reconnect in 2 seconds...")
            time.sleep(2)
        except Exception as e:
            print(f"Unexpected error: {e}")
            time.sleep(2)


thread = threading.Thread(target=read_serial_data, daemon=True)
thread.start()


# Initialize the app
app = Dash(__name__)


# Create control elements for each group
# see [https://dash.plotly.com/tutorial](https://dash.plotly.com/tutorial) for basic informations on plotly (Dash)
control_elements = []
for group_name, variables in GROUPS.items():
    group_div = html.Div([
        html.H5(group_name, style={'font-family': 'Arial', 'marginBottom': '10px'}),
        dcc.Checklist(
            id={'type': 'group-checklist', 'index': group_name},
            options=[{'label': ' ' + NAMES.get(var, var), 'value': var} for var in variables],
            value=variables,  # All selected by default
            style={'font-family': 'Arial'},
            labelStyle={'display': 'block', 'marginBottom': '5px'}
        )
    ], style={
        'backgroundColor': '#f5f5f5', 'padding': '10px', 'borderRadius': '3px'
    })
    control_elements.append(group_div)


app.layout = html.Div([
    html.H2("Prosthetic Sensor Live Telemetry", style={'font-family': 'Arial', 'textAlign': 'center'}),


    # LED Flags Container (Top of the page)
    html.Div(id='led-container', style={
        'display': 'flex', 'justifyContent': 'center', 'flexWrap': 'wrap',
        'gap': '20px', 'marginBottom': '20px', 'font-family': 'Arial'
    }),


    dcc.Graph(id='live-graph', style={'height': '1200px'}), # Taller graph to fit 5 subplots
    # Control Panel for selecting variables (placed below plots)
    html.Div([
        html.H4("Select Data to Display", style={'font-family': 'Arial', 'marginBottom': '15px'}),
        html.Div(control_elements, style={
            'display': 'grid', 'gridTemplateColumns': 'repeat(auto-fit, minmax(200px, 1fr))',
            'gap': '15px', 'marginBottom': '20px'
        })
    ], style={
        'backgroundColor': 'white', 'padding': '15px', 'borderRadius': '5px',
        'marginTop': '10px', 'marginBottom': '20px', 'boxShadow': '0 2px 4px rgba(0,0,0,0.1)'
    }),

    dcc.Interval(id='graph-update', interval=UPDATE_SPEED_MS, n_intervals=0)
], style={'backgroundColor': '#f8f9fa', 'padding': '20px'})



# Helper function to design the LED lights
def create_led(name, state):
    # Default fault flags: Red if 1, Grey if 0
    color = "#ff3333" if state == 1 else "#e0e0e0"
    
    # Special case for "sensors_ok": Green if 1, Red if 0
    if name == "sensors_ok":
        color = "#33cc33" if state == 1 else "#ff3333"

    display_name = FLAG_NAMES.get(name, name)

    return html.Div([
        html.Div(style={
            'width': '24px', 'height': '24px', 'borderRadius': '50%', 
            'backgroundColor': color, 'margin': '0 auto',
            'boxShadow': f'0 0 10px {color}' if state == 1 else 'none'
        }),
        html.Div(display_name, style={'fontSize': '12px', 'marginTop': '5px', 'fontWeight': 'bold'})
    ], style={'textAlign': 'center', 'width': '120px'})



# Update checklist values to keep all sensors selected
@app.callback(
    Output({'type': 'group-checklist', 'index': ALL}, 'value'),
    [Input('graph-update', 'n_intervals')],
    [State({'type': 'group-checklist', 'index': ALL}, 'id')],
    prevent_initial_call=True
)
def update_checklist_values(n, ids):
    global initial_health_check_done

    # Only do this check once at startup when data first arrives
    if initial_health_check_done or not data_queue or n == 0:
        raise PreventUpdate

    initial_health_check_done = True

    df = pd.DataFrame(list(data_queue), columns=COLUMNS)
    sensor_status = check_sensor_status(df)

    updated_values = []
    for id_obj in ids:
        group_name = id_obj['index']
        if group_name in GROUPS:
            # Only keep healthy sensors in this group
            healthy_in_group = [var for var in GROUPS[group_name] if sensor_status.get(var, True)]
            updated_values.append(healthy_in_group)
        else:
            updated_values.append([])

    return updated_values


@app.callback(
    [Output('live-graph', 'figure'),
     Output('led-container', 'children')],
    [Input('graph-update', 'n_intervals'),
     Input({'type': 'group-checklist', 'index': ALL}, 'value')],
    prevent_initial_call=False
)
def update_dashboard(n, selected_lists):
    if not data_queue:
        return go.Figure(), [create_led(f, 0) for f in FLAGS]


    # Flatten the selected variables from all groups
    selected_variables = []
    if selected_lists:
        for var_list in selected_lists:
            if var_list:
                selected_variables.extend(var_list)


    df = pd.DataFrame(list(data_queue), columns=COLUMNS)


    # 1. Update the LED lights based on the single most recent data row
    latest_data = df.iloc[-1]
    led_elements = [create_led(flag, latest_data[flag]) for flag in FLAGS]


    # 2. Update the 5 Subplots - respect user's manual selections
    fig = make_subplots(
        rows=5, cols=1,
        shared_xaxes=True,
        vertical_spacing=0.03,
        subplot_titles=list(GROUPS.keys())
    )


    row_idx = 1
    for title, variables in GROUPS.items():
        for col in variables:
            # Only add trace if it's selected by user (no health filtering after startup)
            if col in selected_variables:
                y_data = df[col]
                # Convert voltage from mV to V
                if col == 'bus_mv':
                    y_data = y_data / 1000
                fig.add_trace(
                    go.Scattergl(
                        x=df['t_ms'] / 1000,
                        y=y_data,
                        mode='lines',
                        name=NAMES.get(col, col)
                    ),
                    row=row_idx, col=1
                )


        # Add invisible dummy trace to keep axes visible even when empty
        fig.add_trace(
            go.Scattergl(
                x=[],
                y=[],
                mode='lines',
                showlegend=False,
                hoverinfo='skip'
            ),
            row=row_idx, col=1
        )
        row_idx += 1


    fig.update_layout(
        template="plotly_white",
        hovermode="x unified",
        margin=dict(l=40, r=40, t=40, b=40),
        legend=dict(orientation="v", yanchor="top", y=1, xanchor="left", x=1.02)
    )


    # Ensure axes sync when zooming in on time
    fig.update_xaxes(matches='x')


    # Fix y-axis ranges for specific subplot types
    # Temp should normally range from 0 to 80, so for this, no need to plot much more
    # fig.update_yaxes(range=[0, 90], row=1, col=1)
    # fig.update_yaxes(range=[0, 90], row=2, col=1)
    # # RH plotted on full range (0 to 100%)
    # fig.update_yaxes(range=[0, 100], row=3, col=1)


    # Ensure axes and grid are always visible, even when empty
    fig.update_xaxes(showgrid=True, showline=True, zeroline=False)
    fig.update_yaxes(showgrid=True, showline=True, zeroline=False)

    # Add x-axis label to bottom subplot
    fig.update_xaxes(title_text="Time [s]", row=5, col=1)


    return fig, led_elements


if __name__ == '__main__':
    # Suppress Werkzeug debug logs to see serial prints cleanly
    logging.getLogger('werkzeug').setLevel(logging.ERROR)


    print("Starting server... Open [http://127.0.0.1:8050](http://127.0.0.1:8050) in your browser.")
    app.run(debug=False)