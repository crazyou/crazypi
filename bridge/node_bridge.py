from __future__ import print_function
import random
import time , threading
import grpc
import crazy_rtc_pb2
import crazy_rtc_pb2_grpc
import sys
import os
import commands 

import rospy
import json
from std_msgs.msg import String

#############################
from PIL import Image
from base64 import standard_b64encode, standard_b64decode
from StringIO import StringIO
from math import floor, ceil, sqrt
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from nav_msgs.msg import OccupancyGrid

from rosbridge_library.internal import ros_loader, message_conversion

import psutil
import urllib
import urllib2
import socket
import fcntl
import struct


try:
    from ujson import dumps
except ImportError:
    try:
        from simplejson import dumps
    except ImportError:
        from json import dumps


def encode(string):
    """ PNG-compress the string in a square RBG image padded with '\n', return the b64 encoded bytes """
    length = len(string)
    width = floor(sqrt(length/3.0))
    height = ceil((length/3.0) / width)
    bytes_needed = int(width * height * 3)
    while length < bytes_needed:
        string += '\n'
        length += 1
    i = Image.frombytes('RGB', (int(width), int(height)), string)
    buff = StringIO()
    i.save(buff, "png")
    encoded = standard_b64encode(buff.getvalue())
    return encoded

def decode(string):
    """ b64 decode the string, then PNG-decompress """
    decoded = standard_b64decode(string)
    buff = StringIO(decoded)
    i = Image.open(buff)
    return i.tostring()

#############################
HB_INTERVAL = 5
SAIYA_SVR = 'www.crazyou.com:10001'
#SAIYA_SVR = '47.254.38.220:10001'
#SAIYA_SVR = '120.76.126.248:10001'
threadLock = threading.Lock()

g_conf = {}
#DID = sys.argv[1]
g_msg_list = []
 
def get_cpu_info():
    with open ('/proc/cpuinfo') as fd:
        for line in fd:
            if line.startswith('model name'):
                cpu_model = line.split(':')[1].strip()
                return cpu_model

def get_local_ip():
    try:
        csock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        csock.connect(('8.8.8.8', 80))
        (addr, port) = csock.getsockname()
        csock.close()
        return addr
    except socket.error:
        return "127.0.0.1"

def get_public_ip():
    url = "https://www.crazyou.cn/get_public_ip"
    req = urllib2.Request(url)
    #print req
    res_data = urllib2.urlopen(req)
    return res_data.read()

def getOsVersion():
    with open('/etc/issue') as fd:
        for line in fd:
            osver = line.strip()
            break
    return osver

def getTotalMemory():
    with open('/proc/meminfo') as fd:
        for line in fd:
            if line.startswith('MemTotal'):
                mem = int(line.split()[1].strip())
                return mem
    #mem = '%.f' % (mem / 1024.0) + ' MB'

def getFreeMemory():
    with open('/proc/meminfo') as fd:
        for line in fd:
            if line.startswith('MemFree'):
                mem = int(line.split()[1].strip())
                return mem
def get_mac():
    for line in os.popen("/sbin/ifconfig"):
        if 'HWaddr' in line:
            nPos = line.index('HWaddr')
            mac = line[nPos+6:] 
            #mac = line.split()[4]
            return (mac.strip())


MAC_LOCAL = get_mac()

def make_rtc_msg(did, cmd, msg, email):
    crazy_msg = crazy_rtc_pb2.CrazyMsg()

    crazy_msg.did = did
    crazy_msg.cmd = cmd
    crazy_msg.msg = msg
    crazy_msg.email = email

    crazy_msg.dinfo.type = crazy_rtc_pb2.ROBOT
    crazy_msg.dinfo.os_info = commands.getoutput('uname -a') 
    crazy_msg.dinfo.mac_addr = MAC_LOCAL
    crazy_msg.dinfo.local_ip = get_local_ip()
    #crazy_msg.dinfo.wifi_ssid = get_wifi_ssid()

    #get cpu info
    cpu_r = psutil.cpu_percent(0.5)
    crazy_msg.dinfo.cpu_rate = cpu_r
    crazy_msg.dinfo.cpu_info = get_cpu_info()
    #disk
    #print ("disk:",psutil.disk_usage('/'))
    #disk = os.statvfs('/')
    #print ("disk2:", disk)

    crazy_msg.dinfo.mem_total = getTotalMemory()
    crazy_msg.dinfo.mem_free = getFreeMemory() 
    #crazy_msg.dinfo.public_ip = get_public_ip()
    return crazy_msg 

def SendSaiya() :
    global g_msg_list
    global g_conf
    hb_last_time = (int)(time.time())
    hb = make_rtc_msg(did=MAC_LOCAL, cmd="HB", msg="", email=g_conf["EMail"])
    g_msg_list.append(hb)
    while 1 :
        while 0 == len(g_msg_list) :
            print("g_msg_list is none, SendSaiya thread=",threading.current_thread().name, threading.currentThread())
            if (int)(time.time()) - hb_last_time > HB_INTERVAL :
                threadLock.acquire()
                #var DeviceInfo dinfo
                #dinfo={cpu_info:"", mac_addr:"", local_ip:"", public_ip:""}
                g_msg_list.insert(0, make_rtc_msg(did=MAC_LOCAL, cmd="HB", msg="", email=g_conf["EMail"]))#, dinfo))
                threadLock.release()
                hb_last_time = (int)(time.time())
                break
            time.sleep(0.3)
        print("g_msg_list len=", len(g_msg_list),"SendSaiya thread=",threading.current_thread().name, threading.currentThread())
        yield g_msg_list.pop()

def CBMap(msg) :
    json_msg = message_conversion.extract_values(msg)
    outgoing_msg = {"op": "publish", "topic": "/map", "msg": json_msg}#.data}
    #print('recv ros map msg:', msg.data)
    print("will deal, Callback thread=",threading.current_thread().name, threading.currentThread())
    outgoing_msg_dumped = dumps(outgoing_msg)
    outgoing_msg = {"op": "png", "data": encode(outgoing_msg_dumped)}
    #print ('map png:[', outgoing_msg,']')
    map_pub.publish(String(dumps(outgoing_msg)))
    #global g_msg_list
    #threadLock.acquire()
    #g_msg_list.insert(0, make_rtc_msg(did=DID, cmd="MAP", msg=outgoing_msg))
    #threadLock.release()

    #g_stub.SendToSaiya()


def CallBackRtc(msg) :
    global g_msg_list
    global g_conf
    print('recv ros msg:', msg.data, "will deal, Callback thread=",threading.current_thread().name, threading.currentThread())
    threadLock.acquire()
    g_msg_list.insert(0, make_rtc_msg(did=MAC_LOCAL, cmd="RTC", msg=msg.data, email=g_conf["EMail"]))
    threadLock.release()
    #g_stub.SendToSaiya()

def GetTwist(linear, angular, vertical):
    twist = Twist()
    twist.linear.x = linear
    twist.angular.z = angular
    twist.angular.x = vertical
    return twist

#callback
def DealRosMsg() :
    r = 0
    while True :
        n = yield r
        if not n :
            print("recv err n=[", n, "].")
            r = -1
            continue
        #ros publish
        #if cmd == n.msg
        if n.cmd == "RTC":
            print("will send cmd:",n.msg ,", Callback thread=",threading.current_thread().name, threading.currentThread())
            rtc_pub.publish(String(n.msg))
        elif n.cmd == "HB":
            pass
        elif n.cmd == "OP":
            print (n)
            twist = Twist()
            arm = Vector3()
            try:
                jd = json.loads(n.msg)
                print (jd)
                if jd['x'] != 0 :
                    twist.linear.x = jd['x']*jd['s']
                if jd['y'] != 0 :
                    twist.angular.z = jd['y']*jd['s']
                    #twist.linear.y = jd.y*jd.s
                if jd['va'] != 0 :
                    arm.z = jd['va']
                if jd['ha'] != 0 :
                    arm.x = jd['ha']

            except :
                print ('except!')
        else:
            print ("cmd err:", n)
        r = 0
        #print('[DealRosMsg] to ROS [%s]' % n)
        #print('DealRosMsg Publish thread =', threading.current_thread().name, threading.currentThread())


def ConnectToSaiya(stub) :
    handle_ros_send.next()
    while 1:
        saiya_responses = stub.Robot2Saiya(SendSaiya())
        for saiya_msg in saiya_responses :
            print("recv saiya_msg =", saiya_msg, ", recv thread=",threading.current_thread().name, threading.currentThread())
            res = handle_ros_send.send(saiya_msg)
            if 0 != res :
                print("deal ros msg result[", res, "]")
        print("recv msg deal done.")
        #time.sleep(1) # ??? need?

def run() :
    channel = grpc.insecure_channel(SAIYA_SVR)
    stub = crazy_rtc_pb2_grpc.SaiyaStub(channel)
    ConnectToSaiya(stub)

if __name__ == '__main__' :
    with open ('conf.txt', 'r') as fo:
        try:
            print("filename: ", fo.name)
            conf_content = fo.read()
        except:
            print("file error!")
    g_conf = json.loads(conf_content)
    print(g_conf["EMail"])
    if "" == g_conf["EMail"]:
        raise "EMail is empty."

    #ros
    rospy.init_node('crazyou_bridge', anonymous=True)
    rospy.Subscriber('crazy_rtc_n', String, CallBackRtc)
    rospy.Subscriber('map', OccupancyGrid, CBMap)
    rtc_pub = rospy.Publisher('crazy_rtc_c', String, queue_size=10)
    map_pub = rospy.Publisher('crazy_map', String, queue_size=10)
    cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    cmd_arm_pub = rospy.Publisher('cmd_arm', Vector3, queue_size=10)

    handle_ros_send = DealRosMsg()
    run()
