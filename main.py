import streamlit as st

st.set_page_config(page_title="RoboLearnX", page_icon="🤖", layout="wide")

st.title("Welcome to RoboLearnX!")
st.write("""
RoboLearnX is your one-stop platform for learning robotics and electronics. 
Explore tutorials, simulations, and learn about electronic components to build your own robots!
""")

st.sidebar.title("Navigation")
page = st.sidebar.radio("Go to", [
    "Home", 
    "Tutorials", 
    "Simulations", 
    "Electronics Components", 
    "Tools", 
    "Resources", 
    "About Us"
])

if page == "Home":
    st.header("Home")
    st.write("""
    **RoboLearnX Features:**
    - **Interactive Tutorials:** Step-by-step guides to learn robotics and electronics.
    - **Simulations:** Run virtual robotics simulations in your browser.
    - **Electronics Components:** Learn about components, their functions, and how to use them.
    - **Tools:** Access circuit design, code editing, and project planning tools.
    - **Resources:** Access additional learning materials and project ideas.
    - **Community Support:** Connect with other learners and experts in the forum.
    """)

elif page == "Tutorials":
    st.header("Tutorials")
    st.write("Explore our step-by-step tutorials to learn robotics and electronics.")

    tutorial_categories = {
        "Python Basics": [
            {
                "title": "Variables and Data Types",
                "content": """
                Learn about variables, integers, floats, and strings in Python.
                - **Variables** are used to store data.
                - **Integers** are whole numbers (e.g., 5).
                - **Floats** are decimal numbers (e.g., 3.14).
                - **Strings** are sequences of characters (e.g., "Hello").
                """,
                "code": """
                x = 5
                y = 3.14
                name = "RoboLearnX"

                print("x:", x)
                print("y:", y)
                print("name:", name)
                """
            },
            {
                "title": "Loops and Conditionals",
                "content": """
                Understand for loops, while loops, and if-else statements in Python.
                - **For loops** iterate over a sequence.
                - **While loops** repeat as long as a condition is true.
                - **If-else statements** make decisions based on conditions.
                """,
                "code": """
                for i in range(5):
                    print("Iteration:", i)

                count = 0
                while count < 3:
                    print("Count:", count)
                    count += 1

                age = 18
                if age >= 18:
                    print("You are an adult.")
                else:
                    print("You are a minor.")
                """
            },
            {
                "title": "Functions and Libraries",
                "content": """
                Learn how to define functions and use Python libraries.
                - **Functions** are reusable blocks of code.
                - **Libraries** are collections of pre-written code.
                """,
                "code": """
                import math

                def calculate_area(radius):
                    return math.pi * radius ** 2

                area = calculate_area(5)
                print("Area:", area)
                """
            },
            {
                "title": "Lists and Dictionaries",
                "content": """
                Learn how to use lists and dictionaries in Python.
                - **Lists** are ordered collections of items.
                - **Dictionaries** are key-value pairs.
                """,
                "code": """
                fruits = ["apple", "banana", "cherry"]
                print(fruits[0])

                person = {"name": "John", "age": 30}
                print(person["name"])
                """
            },
            {
                "title": "File Handling",
                "content": """
                Learn how to read and write files in Python.
                - **Reading Files:** Use the `open()` function.
                - **Writing Files:** Use the `write()` method.
                """,
                "code": """
                with open("example.txt", "r") as file:
                    content = file.read()
                    print(content)

                with open("example.txt", "w") as file:
                    file.write("Hello, RoboLearnX!")
                """
            },
        ],
        "Arduino Programming": [
            {
                "title": "Blinking an LED",
                "content": """
                Learn how to blink an LED using Arduino.
                - Connect the LED to pin 13 and GND.
                - Use the `digitalWrite()` function to turn the LED on and off.
                """,
                "code": """
                void setup() {
                    pinMode(LED_BUILTIN, OUTPUT);
                }

                void loop() {
                    digitalWrite(LED_BUILTIN, HIGH);
                    delay(1000);
                    digitalWrite(LED_BUILTIN, LOW);
                    delay(1000);
                }
                """
            },
            {
                "title": "Reading Sensor Data",
                "content": """
                Learn how to read data from a sensor (e.g., temperature sensor) using Arduino.
                - Connect the sensor to an analog pin.
                - Use the `analogRead()` function to read sensor data.
                """,
                "code": """
                void setup() {
                    Serial.begin(9600);
                }

                void loop() {
                    int sensorValue = analogRead(A0);
                    Serial.println(sensorValue);
                    delay(1000);
                }
                """
            },
            {
                "title": "Controlling a Servo Motor",
                "content": """
                Learn how to control a servo motor using Arduino.
                - Connect the servo to a PWM pin.
                - Use the `Servo` library to control the motor's position.
                """,
                "code": """
                #include <Servo.h>

                Servo myServo;

                void setup() {
                    myServo.attach(9);
                }

                void loop() {
                    myServo.write(0);
                    delay(1000);
                    myServo.write(90);
                    delay(1000);
                    myServo.write(180);
                    delay(1000);
                }
                """
            },
            {
                "title": "Using a Push Button",
                "content": """
                Learn how to use a push button with Arduino.
                - Connect the button to a digital pin.
                - Use the `digitalRead()` function to detect button presses.
                """,
                "code": """
                const int buttonPin = 2;
                const int ledPin = 13;

                void setup() {
                    pinMode(buttonPin, INPUT);
                    pinMode(ledPin, OUTPUT);
                }

                void loop() {
                    if (digitalRead(buttonPin) == HIGH) {
                        digitalWrite(ledPin, HIGH);
                    } else {
                        digitalWrite(ledPin, LOW);
                    }
                }
                """
            },
            {
                "title": "Using a Potentiometer",
                "content": """
                Learn how to use a potentiometer with Arduino.
                - Connect the potentiometer to an analog pin.
                - Use the `analogRead()` function to read the potentiometer value.
                """,
                "code": """
                const int potPin = A0;

                void setup() {
                    Serial.begin(9600);
                }

                void loop() {
                    int potValue = analogRead(potPin);
                    Serial.println(potValue);
                    delay(100);
                }
                """
            },
        ],
        "Advanced Robotics": [
            {
                "title": "PID Control",
                "content": """
                Learn how to implement PID control for robotics applications.
                - **PID (Proportional-Integral-Derivative)** control is used to maintain stability and accuracy.
                - Commonly used in line-following robots, drones, and balancing robots.
                """,
                "code": """
                double Kp = 1.0, Ki = 0.1, Kd = 0.01;
                double error = 0, previous_error = 0, integral = 0, derivative = 0;

                void loop() {
                    error = calculate_error();
                    integral += error;
                    derivative = error - previous_error;
                    double output = Kp * error + Ki * integral + Kd * derivative;
                    previous_error = error;
                    apply_output(output);
                }
                """
            },
            {
                "title": "Path Planning",
                "content": """
                Learn how to implement path planning algorithms for robots.
                - **A* Algorithm:** Finds the shortest path between two points.
                - **Dijkstra's Algorithm:** Finds the shortest path in a weighted graph.
                """,
                "code": """
                def a_star(start, goal):
                    open_set = {start}
                    came_from = {}
                    g_score = {start: 0}
                    f_score = {start: heuristic(start, goal)}

                    while open_set:
                        current = min(open_set, key=lambda x: f_score[x])
                        if current == goal:
                            return reconstruct_path(came_from, current)
                        open_set.remove(current)
                        for neighbor in neighbors(current):
                            tentative_g_score = g_score[current] + distance(current, neighbor)
                            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                                came_from[neighbor] = current
                                g_score[neighbor] = tentative_g_score
                                f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, goal)
                                if neighbor not in open_set:
                                    open_set.add(neighbor)
                    return None
                """
            },
            {
                "title": "Object Detection",
                "content": """
                Learn how to implement object detection using OpenCV.
                - **OpenCV** is a library for computer vision tasks.
                - Use pre-trained models to detect objects in images or video streams.
                """,
                "code": """
                import cv2

                net = cv2.dnn.readNet("yolov3.weights", "yolov3.cfg")
                layer_names = net.getLayerNames()
                output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]

                image = cv2.imread("image.jpg")
                blob = cv2.dnn.blobFromImage(image, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
                net.setInput(blob)
                outs = net.forward(output_layers)

                for out in outs:
                    for detection in out:
                        scores = detection[5:]
                        class_id = np.argmax(scores)
                        confidence = scores[class_id]
                        if confidence > 0.5:
                            center_x = int(detection[0] * image.shape[1])
                            center_y = int(detection[1] * image.shape[0])
                            w = int(detection[2] * image.shape[1])
                            h = int(detection[3] * image.shape[0])
                            x = int(center_x - w / 2)
                            y = int(center_y - h / 2)
                            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                """
            },
            {
                "title": "Robot Localization",
                "content": """
                Learn how to implement robot localization using the Kalman Filter.
                - **Kalman Filter** is used to estimate the state of a system from noisy measurements.
                - Commonly used in robotics for localization and tracking.
                """,
                "code": """
                import numpy as np

                def kalman_filter(x, P, F, H, R, Q, z):
                    x_pred = F @ x
                    P_pred = F @ P @ F.T + Q
                    y = z - H @ x_pred
                    S = H @ P_pred @ H.T + R
                    K = P_pred @ H.T @ np.linalg.inv(S)
                    x_updated = x_pred + K @ y
                    P_updated = (np.eye(len(x)) - K @ H) @ P_pred
                    return x_updated, P_updated
                """
            },
            {
                "title": "SLAM (Simultaneous Localization and Mapping)",
                "content": """
                Learn how to implement SLAM for autonomous robots.
                - **SLAM** allows a robot to build a map of an unknown environment while tracking its location.
                - Commonly used in autonomous vehicles and drones.
                """,
                "code": """
                import numpy as np

                def slam(measurements, motions):
                    x = np.zeros(2)
                    P = np.eye(2)
                    for i in range(len(measurements)):
                        x, P = kalman_filter(x, P, F, H, R, Q, measurements[i])
                        x = x + motions[i]
                    return x, P
                """
            },
        ],
    }

    for category, tutorials in tutorial_categories.items():
        st.subheader(category)
        for tutorial in tutorials:
            with st.expander(tutorial["title"]):
                st.write(tutorial["content"])
                if "code" in tutorial:
                    st.code(tutorial["code"], language="python" if category == "Python Basics" else "cpp")

elif page == "Simulations":
    st.header("Simulations")
    st.write("Run robotics simulations in your browser.")

    st.subheader("Line-Following Robot")
    st.write("Adjust the parameters to see how the robot behaves.")
    speed = st.slider("Robot Speed", 1, 10, 5)
    sensor_sensitivity = st.slider("Sensor Sensitivity", 1, 100, 50)
    if st.button("Run Line-Following Simulation"):
        st.write(f"Running simulation at speed {speed} and sensitivity {sensor_sensitivity}...")
        st.write("Simulation complete!")

    st.subheader("Obstacle Avoidance Robot")
    st.write("Adjust the parameters to see how the robot avoids obstacles.")
    obstacle_distance = st.slider("Obstacle Distance (cm)", 1, 100, 30)
    if st.button("Run Obstacle Avoidance Simulation"):
        st.write(f"Running simulation with obstacle at {obstacle_distance} cm...")
        st.write("Simulation complete!")

    st.subheader("Light-Following Robot")
    st.write("Adjust the parameters to see how the robot follows light.")
    light_intensity = st.slider("Light Intensity", 1, 100, 50)
    if st.button("Run Light-Following Simulation"):
        st.write(f"Running simulation with light intensity {light_intensity}...")
        st.write("Simulation complete!")

    st.subheader("Maze-Solving Robot")
    st.write("Watch the robot solve a maze using pathfinding algorithms.")
    if st.button("Run Maze-Solving Simulation"):
        st.write("Running maze-solving simulation...")
        st.write("Simulation complete!")

    st.subheader("Robotic Arm")
    st.write("Control a robotic arm to pick and place objects.")
    arm_speed = st.slider("Arm Speed", 1, 10, 5)
    if st.button("Run Robotic Arm Simulation"):
        st.write(f"Running robotic arm simulation at speed {arm_speed}...")
        st.write("Simulation complete!")

    st.subheader("Drone Flight Simulation")
    st.write("Simulate the flight of a drone in a virtual environment.")
    altitude = st.slider("Altitude (m)", 1, 100, 10)
    if st.button("Run Drone Flight Simulation"):
        st.write(f"Running drone flight simulation at {altitude} meters...")
        st.write("Simulation complete!")

    st.subheader("Self-Driving Car")
    st.write("Simulate a self-driving car navigating through a city.")
    car_speed = st.slider("Car Speed (km/h)", 10, 100, 50)
    if st.button("Run Self-Driving Car Simulation"):
        st.write(f"Running self-driving car simulation at {car_speed} km/h...")
        st.write("Simulation complete!")

    st.subheader("Robot Vacuum Cleaner")
    st.write("Simulate a robot vacuum cleaner cleaning a room.")
    cleaning_mode = st.selectbox("Cleaning Mode", ["Quick Clean", "Deep Clean"])
    if st.button("Run Robot Vacuum Simulation"):
        st.write(f"Running robot vacuum simulation in {cleaning_mode} mode...")
        st.write("Simulation complete!")

    st.subheader("Autonomous Drone Delivery")
    st.write("Simulate an autonomous drone delivering a package.")
    delivery_distance = st.slider("Delivery Distance (km)", 1, 10, 5)
    if st.button("Run Drone Delivery Simulation"):
        st.write(f"Running drone delivery simulation for {delivery_distance} km...")
        st.write("Simulation complete!")

    st.subheader("Robot Swarm")
    st.write("Simulate a swarm of robots working together.")
    swarm_size = st.slider("Swarm Size", 1, 100, 10)
    if st.button("Run Robot Swarm Simulation"):
        st.write(f"Running robot swarm simulation with {swarm_size} robots...")
        st.write("Simulation complete!")

elif page == "Electronics Components":
    st.header("Electronics Components")
    st.write("Learn about electronic components, their functions, and how to connect them.")

    components = {
        "Basic Components": [
            {
                "name": "Resistor",
                "function": "Limits current flow.",
                "symbol": "R",
                "unit": "Ohm (Ω)",
                "usage": "Used to limit current, divide voltage, and protect components.",
                "connection": "Connect in series with other components.",
                "example_circuit": """
                **Example Circuit: Voltage Divider**
                - Connect two resistors in series.
                - The output voltage is given by: Vout = Vin * (R2 / (R1 + R2))
                """,
                "image": "Resistor.jpg"
            },
            {
                "name": "Capacitor",
                "function": "Stores electrical energy.",
                "symbol": "C",
                "unit": "Farad (F)",
                "usage": "Used in timing circuits, filtering noise, and stabilizing voltage.",
                "connection": "Connect in parallel with a power source.",
                "example_circuit": """
                **Example Circuit: Low-Pass Filter**
                - Connect a resistor and capacitor in series.
                - The capacitor filters out high-frequency signals.
                """,
                "image": "Capacitor.jpg"
            },
            {
                "name": "Diode",
                "function": "Allows current to flow in one direction.",
                "symbol": "D",
                "unit": "N/A",
                "usage": "Used in rectifiers, voltage regulation, and signal demodulation.",
                "connection": "Connect the anode to the positive side and the cathode to the negative side.",
                "example_circuit": """
                **Example Circuit: Half-Wave Rectifier**
                - Connect a diode in series with an AC power source.
                - The diode allows only the positive half of the AC signal to pass.
                """,
                "image": "Diode.jpg"
            },
        ],
        "Sensors": [
            {
                "name": "Ultrasonic Sensor",
                "function": "Measures distance using sound waves.",
                "symbol": "US",
                "unit": "Centimeters (cm)",
                "usage": "Used in obstacle detection, distance measurement, and robotics.",
                "connection": "Connect the Trig pin to a digital output and the Echo pin to a digital input.",
                "example_circuit": """
                **Example Circuit: Distance Measurement**
                - Connect the ultrasonic sensor to an Arduino.
                - Use the `pulseIn()` function to measure the time taken for the sound wave to return.
                """,
                "image": "Ultrasonic Sensor.jpg"
            },
            {
                "name": "Infrared Sensor",
                "function": "Detects obstacles using infrared light.",
                "symbol": "IR",
                "unit": "N/A",
                "usage": "Used in line-following robots, obstacle avoidance, and proximity sensing.",
                "connection": "Connect the output pin to a digital input.",
                "example_circuit": """
                **Example Circuit: Obstacle Detection**
                - Connect the IR sensor to an Arduino.
                - Use the `digitalRead()` function to detect obstacles.
                """,
                "image": "Infrared Sensor.jpg"
            },
            {
                "name": "Light Sensor",
                "function": "Measures light intensity.",
                "symbol": "LS",
                "unit": "Lux (lx)",
                "usage": "Used in automatic lighting systems, light-following robots, and photography.",
                "connection": "Connect the output pin to an analog input.",
                "example_circuit": """
                **Example Circuit: Light Intensity Measurement**
                - Connect the light sensor to an Arduino.
                - Use the `analogRead()` function to measure light intensity.
                """,
                "image": "Light Sensor.jpg"
            },
        ],
        "Actuators": [
            {
                "name": "DC Motor",
                "function": "Converts electrical energy into mechanical motion.",
                "symbol": "M",
                "unit": "RPM (Revolutions Per Minute)",
                "usage": "Used in robotics, drones, and appliances.",
                "connection": "Connect to a motor driver (e.g., L298N) and control with PWM signals.",
                "example_circuit": """
                **Example Circuit: Motor Control**
                - Connect the DC motor to an L298N motor driver.
                - Use PWM signals to control the motor speed.
                """,
                "image": "DC Motor.jpg"
            },
            {
                "name": "Servo Motor",
                "function": "Rotates to a specific angle.",
                "symbol": "SV",
                "unit": "Degrees (°)",
                "usage": "Used in robotic arms, camera mounts, and RC vehicles.",
                "connection": "Connect to a PWM pin and control with the `Servo` library.",
                "example_circuit": """
                **Example Circuit: Servo Control**
                - Connect the servo motor to an Arduino.
                - Use the `Servo` library to control the motor's position.
                """,
                "image": "Servo Motor.jpg"
            },
            {
                "name": "Stepper Motor",
                "function": "Moves in precise steps.",
                "symbol": "SM",
                "unit": "Steps per Revolution",
                "usage": "Used in 3D printers, CNC machines, and robotics.",
                "connection": "Connect to a stepper motor driver (e.g., A4988) and control with step and direction signals.",
                "example_circuit": """
                **Example Circuit: Stepper Motor Control**
                - Connect the stepper motor to an A4988 driver.
                - Use step and direction signals to control the motor.
                """,
                "image": "Stepper Motor.jpg"
            },
        ],
    }

    for category, component_list in components.items():
        st.subheader(category)
        cols = st.columns(3)
        for i, component in enumerate(component_list):
            with cols[i % 3]:
                st.image(component["image"], use_container_width=True)
                st.write(f"**{component['name']}**")
                st.write(f"**Function:** {component['function']}")
                st.write(f"**Symbol:** {component['symbol']}")
                st.write(f"**Unit:** {component['unit']}")
                st.write(f"**Usage:** {component['usage']}")
                st.write(f"**Connection:** {component['connection']}")
                with st.expander("Example Circuit"):
                    st.write(component["example_circuit"])

elif page == "Tools":
    st.header("Tools")
    st.write("Access useful tools for robotics and electronics projects.")

    with st.expander("Circuit Designer"):
        st.write("""
        **Design and simulate circuits:**
        - Drag and drop components to create circuits.
        - Simulate the circuit to test its functionality.
        """)
        if st.button("Open Circuit Designer"):
            st.write("Circuit Designer is under development. Stay tuned!")

    with st.expander("Code Editor"):
        st.write("""
        **Write and test code in the browser:**
        - Supports Python, Arduino, and other languages.
        - Real-time syntax highlighting and error checking.
        """)
        if st.button("Open Code Editor"):
            st.write("Code Editor is under development. Stay tuned!")

    with st.expander("Project Planner"):
        st.write("""
        **Plan and organize robotics projects:**
        - Create task lists and timelines.
        - Track progress and manage resources.
        """)
        if st.button("Open Project Planner"):
            st.write("Project Planner is under development. Stay tuned!")

elif page == "Resources":
    st.header("Resources")
    st.write("Access additional learning materials and project ideas.")
    st.write("""
    - **Books:** "Robotics for Beginners" by John Doe
    - **Websites:** [Arduino Official Website](https://www.arduino.cc/)
    - **Videos:** YouTube tutorials on robotics and electronics
    - **Projects:** Build a line-following robot, obstacle-avoidance robot, and more!
    """)

elif page == "About Us":
    st.header("About Us")
    st.write("""
    RoboLearnX is developed by **Ismahan Adam** to make robotics and electronics learning accessible to everyone.
    Our mission is to inspire the next generation of innovators by providing high-quality tutorials, simulations, and resources.
    """)

st.sidebar.markdown("---")
st.sidebar.write("Developed by **Ismahan Adam**")
st.sidebar.write("© 2025 RoboLearnX. All rights reserved.")