# Simple test for NeoPixels on Raspberry Pi
#from fusion import Fusion
import numpy as np
#import utime as time
import time
import threading
import board
import neopixel
import paho.mqtt.client as mqtt
from datetime import datetime
from fastdtw import fastdtw
# Choose an open pin connected to the Data In of the NeoPixel strip, i.e. board.D18
# NeoPixels must be connected to D10, D12, D18 or D21 to work.
pixel_pin = board.D18
def time_diff(t2,t1):
  return t2-t1
# The number of NeoPixels
num_pixels = 30
num_players = 2
# The order of the pixel colors - RGB or GRB. Some NeoPixels have red and green reversed!
# For RGBW NeoPixels, simply change the ORDER to RGBW or GRBW.
ORDER = neopixel.GRB
MQTT_SERVER = "localhost"

pixels = neopixel.NeoPixel(pixel_pin, num_pixels, brightness=0.2, auto_write=False,
                           pixel_order=ORDER)
lookup_player_id = ['player{}'.format(i) for i in range(num_players)]
red = threading.Event()
win_size = 5
euler = np.zeros((num_players,win_size,3))
id_window = 0
t=[0]*num_players
#sensors = [Fusion(time_diff) for i in range(num_players)]
n_calibration = 200
idx = 0
accel_data = np.zeros((num_players,win_size,3))
diff_total_buffer = [0]*win_size
diff_total_smooth = 0
RED_THRESHOLD = 8


def wheel(pos):
    # Input a value 0 to 255 to get a color value.
    # The colours are a transition r - g - b - back to r.
    if pos < 0 or pos > 255:
        r = g = b = 0
    elif pos < 85:
        r = int(pos * 3)
        g = int(255 - pos*3)
        b = 0
    elif pos < 170:
        pos -= 85
        r = int(255 - pos*3)
        g = 0
        b = int(pos*3)
    else:
        pos -= 170
        r = 0
        g = int(pos*3)
        b = int(255 - pos*3)
    return (r, g, b) if ORDER == neopixel.RGB or ORDER == neopixel.GRB else (r, g, b, 0)


def rainbow_cycle(wait):
    for j in range(255):
        for i in range(num_pixels):
            pixel_index = (i * 256 // num_pixels) + j
            pixels[i] = wheel(pixel_index & 255)
        pixels.show()
        time.sleep(wait)

def set(r,g,b):
    for i in range(num_pixels):
        pixels[i] = (r,g,b)
    #times.sleep(wait)	


def subscribe_to_topics(client): 
    for i in range(num_players):
       client.subscribe("theguy/player{}".format(i))
       print("Subscribed to theguy/player{}".format(i))
 
# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
        print("Connected with result code "+str(rc))
        subscribe_to_topics(client)

 
# The callback for when a PUBLISH message is received from the server.
#offset = np.zeros((num_players,n_calibration,9))

def get_euler_0():
  return euler[0,idx]


def get_euler_1():
  return euler[1,idx]

def calibrating():
  return idx < 200

get_euler = [
get_euler_0,
get_euler_1
]

def on_message(client, userdata, msg):
    global win_size, euler, id_window, t, sensors, num_players, idx, diff_total_smooth
    ########print(msg.topic+" "+str(msg.payload))

    source = str(msg.topic).split('/')
    #######print(source)
    topic,id_player = source

    topic = str(topic)
    id_player = int(lookup_player_id.index(id_player))
    values = [float(n) for n in str(msg.payload)[2:-1].split(',')]

    ##########print('Topic: {}, Player: {}, Value: {}'.format(topic,id_player,values))

    ts = values[-1]
    #sensors[id_player].calibrate(get_euler[id_player],calibrating)
#    sensors[id_player].update(values[:3],values[3:6],values[6:9],ts)
#     sensors[id_player].update_nomag(values[:3]-offset[:3],values[3:6]-offset[3:6],ts)

#    for i in range(num_players):
#      euler[i,idx%win_size,0] = sensors[i].roll
#      euler[i,idx%win_size,1] = sensors[i].pitch
#      euler[i,idx%win_size,2] = sensors[i].heading
#      #euler[i,idx] -= offset[id_player]

    accel_data[id_player,idx%win_size] = values[:3]

      #print(np.round(euler,2))



    ###########print("\nPlayer 1: {0:.2f}|{1:.2f}|{2:.2f} Player 2: {3:.2f}|{4:.2f}|{5:.2f}".format(
    # accel_data[0,idx%win_size,0],
    # accel_data[0,idx%win_size,1],
    # accel_data[0,idx%win_size,2],
    # accel_data[1,idx%win_size,0],
    # accel_data[1,idx%win_size,1],
    # accel_data[1,idx%win_size,2]))


    diff_total_x = np.abs(accel_data[0,idx%win_size,0] - accel_data[1,idx%win_size,0])
    diff_total_y = np.abs(accel_data[0,idx%win_size,1] - accel_data[1,idx%win_size,1])
    diff_total_z = np.abs(accel_data[0,idx%win_size,2] - accel_data[1,idx%win_size,2])
    diff_total   = np.sqrt(diff_total_x ** 2 + diff_total_y ** 2 + diff_total_z ** 2)

    diff_total_buffer[idx] = diff_total
    diff_total_smooth = np.mean(diff_total_buffer, 0)

    # print("x: {}, y: {}, z: {}, tot: {}, sm: {}".format(diff_total_x, diff_total_y, diff_total_z, diff_total, diff_total_smooth))
    



#    diff_total = np.linalg.norm(euler[0]-euler[1])
#    diff_current = np.linalg.norm(euler[0,idx%win_size]-euler[1,idx%win_size])

#    dtw_diff,_ = fastdtw(euler[0],euler[1])
    idx = (idx + 1) % win_size

#    print('Eucl. Distance Current: {}'.format(diff_current))
#    print('Eucl. Distance Window: {}'.format(diff_total))

#    print('Time Warped Distance: {}'.format(dtw_diff))

    if diff_total_smooth > RED_THRESHOLD:
        red.set()
        # print("Out of Sync")
    else:
        # print("Clear")
        red.clear()

#    with open('outfile.csv', 'a') as f:
#       f.write('{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}\n'.format(
#        sensors[0].ax,
#        sensors[0].ay,
#        sensors[0].az,
#        sensors[0].gx,
#        sensors[0].gy,
#        sensors[0].gz,
#        sensors[0].mx,
#        sensors[0].my,
#        sensors[0].mz,
#        sensors[0].roll,
#        sensors[0].pitch,
#        sensors[0].heading,
#        sensors[1].ax,
#        sensors[1].ay,
#        sensors[1].az,
#        sensors[1].gx,
#        sensors[1].gy,
#        sensors[1].gz,
#        sensors[1].mx,
#        sensors[1].my,
#        sensors[1].mz,
#        sensors[1].roll,
#        sensors[1].pitch,
#        sensors[1].heading,
#        ))





def main(): 

    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
     
    client.connect(MQTT_SERVER, 1883, 60)
     
    # Blocking call that processes network traffic, dispatches callbacks and
    # handles reconnecting.
    # Other loop*() functions are available that give a threaded interface and a
    # manual interface.
    j = 0
    while True:
    # COMM
        client.loop(.1)
    

    # LED
        j += 20
        if j > 140:
            j = 0

        gradient = diff_total_smooth * 255 / RED_THRESHOLD
        if (gradient > 255): gradient = 255
        set(int(gradient), int(255 - gradient), 0)
        pixels.show()

        # if not red.isSet():
        #     for i in range(num_pixels):
        #         pixel_index = (i * 256 // num_pixels) + j
        #         pixels[i] = wheel(pixel_index & 255)
        #     pixels.show()
        # else:
        #     set(255,0,0)
        #     pixels.show()




main()
