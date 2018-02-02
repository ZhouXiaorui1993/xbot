#!/usr/bin/env python
#coding=utf-8

import time,cv2
import os
import base64
import urllib2
from json import *

url = "http://172.16.0.143:8000/management/register?method=normal"
cap = cv2.VideoCapture(0)
body = str()
post_data = dict()

while(True):
	# Capture frame-by-frame
	ret, frame = cap.read()

	# Our operations on the frame com`e here
	# gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

	# Display the resulting frame
	cv2.imshow('frame',frame)
	if cv2.waitKey(1) & 0xFF == ord('r'):
		cv2.imwrite('tmp.jpg',frame)
		tip = 'please input your name in pinyin:\n'
		name = input(tip)
		post_data["Userid"] = name
		with open('tmp.jpg', "rb") as fp:
			image_binary = fp.read()
			image_binary = base64.b64encode(image_binary)
			post_data["Image"] = image_binary
		if post_data:
				body = JSONEncoder().encode(post_data)
		req = urllib2.Request(url, body)
		response = urllib2.urlopen(req)
		buf = response.read()
		body = JSONDecoder().decode(buf)
		print body["Ret"]
	elif cv2.waitKey(1) & 0xFF == ord('q'):
		break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()


