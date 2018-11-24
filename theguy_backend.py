# Simple test for NeoPixels on Raspberry Pi
from fusion import Fusion
import numpy as np
#import utime as time
import time
import threading
import board
import neopixel
import paho.mqtt.client as mqtt
from datetime import datetime
from fastdtw import fastdtw
#from scipy.spatial.distance import euclidian
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

pixels = neopixel.NeoPixel(pixel_pin, num_pixels, brightness=0.2, auto_write=False,
                           pixel_order=ORDER)
lookup_player_id = ['player{}'.format(i) for i in range(num_players)]
red = threading.Event()
win_size = 10
euler = np.zeros((num_players,win_size,3))
id_window = 0
t=0
sensors = [Fusion(time_diff) for i in range(num_players)]



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


def on_message(client, userdata, msg):
    global win_size, euler, id_window, t, sensors, num_players
    print(msg.topic+" "+str(msg.payload))

    source = str(msg.topic).split('/')
    print(source)
    topic,id_player = source

    topic = str(topic)
    id_player = int(lookup_player_id.index(id_player))
    values = [float(n) for n in str(msg.payload)[2:-1].split(',')]

    print('Topic: {}, Player: {}, Value: {}'.format(topic,id_player,values))

    sensors[id_player].update_nomag(values[:3],values[3:],datetime.now().microsecond)

    idx = t%win_size
    for i in range(num_players):
      euler[i,idx,0] = sensors[i].roll
      euler[i,idx,1] = sensors[i].pitch
      euler[i,idx,2] = sensors[i].heading

    print(np.round(euler,2))
    print("\nPlayer 1: {0:02f}|{1:02f}|{2:02f} Player 2: {3:02f}|{4:02f}|{5:02f}".format(
      euler[0,idx,0],
      euler[0,idx,1],
      euler[0,idx,2],
      euler[1,idx,0],
      euler[1,idx,1],
      euler[1,idx,2]))

    diff_total = np.linalg.norm(euler[0]-euler[1])
    diff_current = np.linalg.norm(euler[0,idx]-euler[1,idx])

    dtw_diff,_ = fastdtw(euler[0],euler[1])
    t+=1

    print('Diff Current: {}'.format(diff_current))
    print('Diff Total: {}'.format(diff_total))

    print('DTW Diff: {}'.format(dtw_diff))

    if diff_total > 100 :
       red.set()
       print("Out of Sync")
    else:
       print("Clear")
       red.clear()

MQTT_SERVER = "localhost"
 
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
#

# LED
    j += 20
    if j > 255:
        j = 0
    if not red.isSet():
        for i in range(num_pixels):
            pixel_index = (i * 256 // num_pixels) + j
            pixels[i] = wheel(pixel_index & 255)
        pixels.show()
    else:
        set(255,0,0)
        pixels.show()
# COMM
    client.loop(.1)

