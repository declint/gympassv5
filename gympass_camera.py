# Använder python 2.7
# Script for capturing images
#
# sudo apt-get install python-picamera python-requests pika
#
# Arduino URL
# http://intranet.strongest.se/gympassv4/index.php/doorpassV5/rmq_store_message_to_queue/doorpass_skagget_picture/tag_en_bild
#

import requests
import MySQLdb
import time
import picamera
import pika

passage_point = "SKAGGET"
send_url = 'http://intranet.strongest.se/gympassv4/index.php/DoorpassWebcam/storewebcamsnap'
rmq_queue_name = 'doorpass_skagget_picture'


def getpassageno():
	r = requests.get('http://intranet.strongest.se/gympassv4/index.php/DoorpassWebcam/getlastpassage/' + passage_point, auth=('gym', 'muskler'))
	return r.text.split(":")[1]

def picam_snaps():
#	camera.start_preview()
#	time.sleep(2)

	camera.led = True

	print("Capturing")
	millis_start = int(round(time.time() * 1000))
	camera.capture_sequence([
		'pycam_image_01.jpg',
		'pycam_image_02.jpg',
		'pycam_image_03.jpg',
		'pycam_image_04.jpg',
		'pycam_image_05.jpg'
	])#, use_video_port=True)#, burst=True)
		
	millis_end = int(round(time.time() * 1000))
	print(millis_end - millis_start)
	
	camera.led = False

#	camera.stop_preview()


def send_files(id):
	millis_start = int(round(time.time() * 1000))
	print("Sending images to server")
	
	files = {'01':open('pycam_image_01.jpg','rb'),
			 '02':open('pycam_image_02.jpg','rb'),
			 '03':open('pycam_image_03.jpg','rb'),
			 '04':open('pycam_image_04.jpg','rb'),
			 '05':open('pycam_image_05.jpg','rb')}
	
	r = requests.post(send_url, files=files, auth=('gym', 'muskler'), data = {"passage_id":str(id)})
	millis_end = int(round(time.time() * 1000))
	print(r.text)
	print("Time to send:" + str(millis_end - millis_start) + "ms")


# Callback to take a snap
def callback_mq(ch, method, properties, body):
	current_id = getpassageno()
	print ("Detected passage with ID:" + str(current_id))
	picam_snaps()
	send_files(current_id)

# Setup RabbitMQ
credentials = pika.PlainCredentials('doorpass', 'doorpass')
connection = pika.BlockingConnection(pika.ConnectionParameters('virtualpt.se', 5672,'/',credentials))
channel = connection.channel()
channel.queue_declare(queue=rmq_queue_name)

channel.basic_consume(callback_mq,
                      queue=rmq_queue_name,
                      no_ack=True)

#Setup camera
camera = picamera.PiCamera()
camera.hflip = True
camera.vflip = True
camera.resolution = (1296, 972)
#camera.resolution = (2592, 1944)
camera.framerate = 5

last_passageid = 0

print ("Starting to look for passage")
channel.start_consuming()

