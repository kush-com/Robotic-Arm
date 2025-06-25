import cv2
import mediapipe as mp
import serial
import time
import paho.mqtt.client as mqtt

# MQTT broker address and port
broker_address = "192.168.232.169"
broker_port = 1883

#create mqtt client
client = mqtt.Client()

#connect to mqtt broker
client.connect(broker_address, broker_port)

#subscribe to a topic
topic = "Python"
client.subscribe(topic)




#  SERIAL SETUP ←
# arduino = serial.Serial('COM5', 9600, timeout=1)
# time.sleep(2)

# → MEDIAPIPE SETUP ←
mp_hands = mp.solutions.hands
hands    = mp_hands.Hands(min_detection_confidence=0.5,
                          min_tracking_confidence=0.5)

cap = cv2.VideoCapture(0)
t0  = time.time()

# calibration storage
diff1 = {}   # open hand diffs
diff2 = {}   # closed hand diffs

open_captured   = False
closed_captured = False

# utility: linear‐map dy→angle
def make_mapper(y1, a1, y2, a2):
    m = (a2 - a1) / (y2 - y1)
    c = a1 - m * y1
    return lambda y: int(round(m * y + c))

def low_pass_filter(prev_value, new_value, alpha=0.2):
    return alpha * new_value + (1 - alpha) * prev_value
filtered_thumb_x_diff = 0  # Initialize somewhere globally or per finger


while True:
    ret, frame = cap.read()
    if not ret:
        break

    elapsed = time.time() - t0
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results   = hands.process(frame_rgb)

    # decide which message to show
    if elapsed < 4:
        msg = "SHOW OPEN FIST for 5s"
    else:
        msg = "SHOW CLOSED FIST for 5s"
    cv2.putText(frame, msg, (10,30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)

    # once we see landmarks, grab diffs at t≈3s (open) and t≈7s (closed)
    if results.multi_hand_landmarks:
        lm = results.multi_hand_landmarks[0].landmark

        # compute all diffs
        y_diff = {
            'thumb' : lm[4].y - lm[2].y,
            'index' : lm[8].y - lm[5].y,
            'middle': lm[12].y - lm[9].y,
            'little': lm[20].y - lm[17].y
        }
        x_diff_thumb = lm[2].x - lm[0].x
       

        # capture open at ~3s
        if not open_captured and elapsed >= 3:
            diff1 = y_diff.copy()
            diff1['x_thumb'] = x_diff_thumb
            open_captured   = True
            print("OPEN diffs:", diff1)

        # capture closed at ~7s
        if open_captured and not closed_captured and elapsed >= 7:
            diff2 = y_diff.copy()
            diff2['x_thumb'] = x_diff_thumb
            closed_captured = True
            print("CLOSED diffs:", diff2)

    cv2.imshow("Calibration", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    

    # once both captured, break out to control loop
    if open_captured and closed_captured and elapsed > 8:
        break

cap.release()
cv2.destroyAllWindows()

# → BUILD MAPPERS ←
# thumb twist: (x1,180),(x2,110)
map_twist  = make_mapper(diff1['x_thumb'], 180,
                         diff2['x_thumb'], 80)
# thumb Y:     (y1,120),(y2,  0) 
map_thumbY = make_mapper(diff1['thumb'], 120,
                         diff2['thumb'],  0)
# index Y:     (y1,130),(y2, 20)
map_indexY = make_mapper(diff1['index'], 120,
                         diff2['index'],  0)
# middle Y:    (y1,150),(y2, 10)
map_midY   = make_mapper(diff1['middle'], 150,
                         diff2['middle'],  10)
# little Y:    (y1,150),(y2, 30)
map_little = make_mapper(diff1['little'], 170,
                         diff2['little'], 40)

print("Mappers ready. Entering control loop…")

# → CONTROL LOOP ←
cap = cv2.VideoCapture(0)
while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results   = hands.process(frame_rgb)

    if results.multi_hand_landmarks:
        lm = results.multi_hand_landmarks[0].landmark

        # current diffs
        x_t = lm[2].x - lm[0].x
        y_t = lm[4].y - lm[2].y
        y_i = lm[8].y - lm[5].y
        y_m = lm[12].y - lm[9].y
        y_l = lm[20].y - lm[17].y

         # Inside your processing loop:
        
        filtered_thumb_x_diff = low_pass_filter(filtered_thumb_x_diff, x_t, alpha=0.2)

        # compute angles
        a_twist  = map_twist(x_t)
        a_thumbY = map_thumbY(y_t)
        a_index  = map_indexY(y_i)
        a_middle = map_midY(y_m)
        a_little = map_little(y_l)

        msg = f"{a_twist},{a_thumbY},{a_index},{a_middle},{a_little}\n"
        # arduino.write(msg.encode())

        #publishing the same string to mqtt broker, i.e ESP32
        client.publish(topic, msg)
        print(f"Published message: {msg}")

        time.sleep(0.05)
        # time.sleep(1)
        # debug print
        cv2.putText(frame, 
            f"T:{a_twist} TY:{a_thumbY} I:{a_index}",
            (10,60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,0),1)
        cv2.putText(frame, 
            f"M:{a_middle} L:{a_little}",
            (10,85), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,0),1)
    # print(msg)
    cv2.imshow("Control", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
# arduino.close()