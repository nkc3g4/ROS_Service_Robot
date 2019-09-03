# -*- coding: utf-8 -*-
from Tkinter import *

import rospy
import sys, os, time

import numpy as np
import dlib

import theano
import theano.tensor as T

import lasagne
from lasagne.updates import adam
from lasagne.layers import DenseLayer, InputLayer, get_output, \
                           get_all_params, set_all_param_values, get_all_param_values
from lasagne.nonlinearities import rectify, softmax
from lasagne.objectives import categorical_crossentropy

import cv2
from cv_bridge import CvBridge, CvBridgeError
from rospy.exceptions import ROSException

from sensor_msgs.msg import Image
from face_recognition_msgs.msg import DetectedFaces

import rospkg

import std_msgs
from std_msgs.msg import String

flag = -1
doublecheck = -1
global framecount
global ori_image
framecount = 0
faceVR = rospy.Publisher('/voice_system/tts_topic',String,queue_size=0)

pkg_path = rospkg.RosPack().get_path('face_recognition')

class FaceRecognition:
    def __init__(self,
                 image_topic       = '/camera/rgb/image_raw',
                 predictor_model   = pkg_path + '/models/shape_predictor_68_face_landmarks.dat',
                 recognition_model = pkg_path + '/models/dlib_face_recognition_resnet_model_v1.dat',
                 database_names    = os.listdir(pkg_path + '/training_data'),
                 n_input           = 128,
                 n_output          = len(os.listdir(pkg_path + '/training_data')),
                 load_model        = True,
                 prob_threshhold   = 0.97):


        # define global variables
        self.n_input        = n_input
        self.n_output       = n_output
        self.database_names = np.array(database_names)
        self.prob_threshold = prob_threshhold
        self.doublecheck    = doublecheck
        self.flag           = flag


        # define network for face prediction
        def model(embedding):
            l_in = InputLayer(input_var = embedding,
                              shape     = (None, n_input))

            h_1  = DenseLayer(l_in,
                              num_units    = 64,
                              nonlinearity = rectify)

            h_2  = DenseLayer(h_1,
                              num_units    = 32,
                              nonlinearity = rectify)

            probs = DenseLayer(h_1,
                               num_units    = n_output,
                               nonlinearity = softmax)

            return probs

        self.X = T.fmatrix()
        self.Y = T.fmatrix()

        self.network_probs = model(self.X)
        

        # load the trained weights for the network
        try:
            if load_model:
                with np.load(pkg_path + '/models/model.npz') as f:
                    param_values = [f['arr_%d' % i] for i in range(len(f.files))]
                    set_all_param_values(self.network_probs, param_values)
        except IOError:
            print 'run the training script before use!'


        # define the network functions
        self.output = get_output(self.network_probs)

        self.predict_faces = theano.function(inputs               = [self.X],
                                             outputs              = self.output,
                                             allow_input_downcast = True)


        # define all face recognition functions
        self.face_detector          = dlib.get_frontal_face_detector()
        self.face_pose_predictor    = dlib.shape_predictor(predictor_model)
        self.face_recognition_model = dlib.face_recognition_model_v1(recognition_model)


        # start the ros pipeline
        self.face_features_pub     = rospy.Publisher('face_recognition/marked_faces', Image, queue_size=0)
        self.face_labels_pub       = rospy.Publisher('face_recognition/labelled_faces', Image, queue_size=0)
        self.face_msgs_pub         = rospy.Publisher('face_recognition/detected_faces', DetectedFaces, queue_size=0)

       # unknownfaceVR 	   = rospy.Publisher('/voice_system/tts',String,queue_size=0)

        self.bridge = CvBridge()

        self.camera_sub = rospy.Subscriber(image_topic, Image, self.camera_cb)
        self.flag = 0
        def mkdir(path):
            folder = os.path.exists(path)
            if not folder:                   #判断是否存在文件夹如果不存在则创建为文件夹
                os.makedirs(path)            #makedirs 创建文件时如果路径不存在会创建这个路径

        def printInfo():
            mkdir("/home/sz/omniWheelCareRobot/rosCode/src/face_recognition/face_recognition/training_data/"+entry1.get())

            global ori_image
            
            cv2.imwrite("/home/sz/omniWheelCareRobot/rosCode/src/face_recognition/face_recognition/training_data/"+entry1.get()+"/"+entry1.get()+"_1.jpg",ori_image)
            print("Image Saved!")
        def trainData():
            os.system("gnome-terminal -x python2 /home/sz/omniWheelCareRobot/rosCode/src/face_recognition/face_recognition/scripts/train_face_recognition.py exit")
            
        #初始化Tk()
        myWindow = Tk()
        #设置标题
        myWindow.title('Python GUI Learning')
        #标签控件布局
        Label(myWindow, text="Name").grid(row=0)
        Label(myWindow, text="--").grid(row=1)
        #Entry件布局
        entry1=Entry(myWindow)
        entry2=Entry(myWindow)
        entry1.grid(row=0, column=1)
        entry2.grid(row=1, column=1)
        #Quit按钮退出；Run按钮打印计算结果
        Button(myWindow, text='Train', command=trainData).grid(row=2, column=0, sticky=W, padx=5,pady=5)
        Button(myWindow, text='Capture', command=printInfo).grid(row=2, column=1, sticky=W, padx=5, pady=5)
        #进入消息循环
        if load_model:
            myWindow.mainloop()
    
    def camera_cb(self,
                  data):
	global framecount
	framecount+=1
	if framecount <10:
	    return
	framecount=0
        faces_detected = DetectedFaces()
        faces_detected.header = data.header

        # define the incoming message as a cv_image
        try:
            image = np.array(self.bridge.imgmsg_to_cv2(data, "bgr8"))
        except CvBridgeError as e:
            print e

        features_image = image.copy()
        labels_image   = image.copy()
        global ori_image
        ori_image = image.copy()
        detected_faces = self.face_detector(image, 1)
        
        for i, face_rect in enumerate(detected_faces):
            # Get bounding box coordinates
            bbox_points    = np.array([face_rect.left(), face_rect.top(), face_rect.right(), face_rect.bottom()])

            # Get the the face's pose
            pose_landmarks = self.face_pose_predictor(image, face_rect)
            pose_points    = np.array([[p.x, p.y] for p in pose_landmarks.parts()])

            # using resnet, find the face embeddings
            embeddings       = np.array(self.face_recognition_model.compute_face_descriptor(image, pose_landmarks, 1))
            prediction_probs = self.predict_faces([embeddings])
            max_prob         = max(prediction_probs[0])
            prediction_index = np.argmax(prediction_probs, axis = 1)
            predicted_name   = self.database_names[prediction_index][0]
            prediction_dist  = np.dot(prediction_probs[0], prediction_probs[0])

            # add information to the ROS message
            faces_detected.names.append(predicted_name)
            faces_detected.probability.append(max_prob)
            faces_detected.distances.append(prediction_dist)

            try:
                for (x, y) in pose_points:
                    cv2.circle(features_image, (x, y), 1, (255, 255, 255), -1)
            except UnboundLocalError:
                pass

            if max_prob > self.prob_threshold:
                try:
                    cv2.putText(labels_image, predicted_name, (bbox_points[0], bbox_points[3]), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,255),3)
                except UnboundLocalError:
                    pass
           
            projectThreshhold = 0.8
            projectCountLimit = 5 
            rospy.loginfo(self.flag)           
            
            if max_prob < projectThreshhold and self.flag == 0:
                    faceVR.publish(std_msgs.msg.String("检测到陌生人"))
                    self.flag = 1
                    self.doublecheck = 1
	    
            elif max_prob >= projectThreshhold:
            	if max_prob > 0.999 and self.doublecheck == 1:
    	            if predicted_name == "wzh":

	                faceVR.publish(std_msgs.msg.String("人脸识别成功, 欢迎您王子豪"))
		    self.doublecheck = 0
	           # if predicted_name == "gy":
                   #     faceVR.publish(std_msgs.msg.String("人脸识别成功, 欢迎您高源"))

	 #           if predicted_name == "wzh":
         #               faceVR.publish(std_msgs.msg.String("人脸识别成功, 欢迎您王子豪"))
		self.flag = 0
            
                

        try:
            self.face_msgs_pub.publish(faces_detected)
            try:
                self.face_features_pub.publish(self.bridge.cv2_to_imgmsg(features_image, "bgr8"))
                self.face_labels_pub.publish(self.bridge.cv2_to_imgmsg(labels_image, "bgr8"))
            except CvBridgeError as e:
                print e
        except ROSException:
            pass


    def get_data(self):
        embeddings = []
        outputs    = []

        train_folders = os.listdir(pkg_path + '/training_data')

        for k, folder in enumerate(train_folders):
            subfolder = pkg_path + '/training_data/' + folder
            for j, image_name in enumerate(os.listdir(subfolder)):
                # Load the image
                file_name = subfolder + '/' + image_name
                image = cv2.imread(file_name)

                # Run the HOG face detector on the image data
                detected_faces = self.face_detector(image, 1)

                print("Found {} faces in the image file {}".format(len(detected_faces), file_name))

                # Loop through each face we found in the image
                for i, face_rect in enumerate(detected_faces):
                    # Get the the face's pose
                    pose_landmarks = self.face_pose_predictor(image, face_rect)

                    embeddings.append(np.array(self.face_recognition_model.compute_face_descriptor(image, pose_landmarks, 1)))
                    outputs.append(np.eye(self.n_output)[k])

        return embeddings, outputs


    def train_network(self,
                      n_epochs      = 10000,
                      learning_rate = 0.001):

        loss = categorical_crossentropy(self.output, self.Y)
        loss = loss.mean()

        params = get_all_params(self.network_probs,
                                trainable = True)

        updates = adam(loss,
                       params,
                       learning_rate = learning_rate)

        train = theano.function(inputs               = [self.X, self.Y],
                                outputs              = loss,
                                updates              = updates,
                                allow_input_downcast = True)

        trX, trY = self.get_data()

        for epoch in range(n_epochs):
            train_loss = train(trX, trY)
            if epoch%50 == 0:
                print 'epoch: %d, loss: %f' % (epoch, train_loss)

        np.savez(pkg_path + '/models/model.npz', *get_all_param_values(self.network_probs))
