##############################################################################
#
#	Executing the SVM classifier on dataset of positive and negative classes
#
##############################################################################


import cv2
import numpy as np
import glob

from svm import *
import svmutil as svms

posNames = glob.glob("pos/*.jpg")
negNames = glob.glob("neg/*.jpg")

hog = cv2.HOGDescriptor()

#'''
posNameSize = len(posNames)
negNameSize = len(negNames)

features = np.zeros((posNameSize+negNameSize,3780),dtype = np.float)
ones = np.ones((posNameSize,1),dtype = np.float)
zeros = np.zeros((negNameSize,1),dtype = np.float)
labels = np.concatenate((ones,zeros)).ravel()

for i in range(0,posNameSize):
    img = cv2.imread(posNames[i],0)
    img = cv2.resize(img,(64,128))
    feature = hog.compute(img)
    feature = np.transpose(feature);
    features[i] = feature
    

for i in range(0,negNameSize):
    img = cv2.imread(negNames[i],0)
    img = cv2.resize(img,(64,128))
    feature = hog.compute(img)
    feature = np.transpose(feature);
    features[i+posNameSize] = feature
    
print " ------ Feature Extraction Done -------"

#m = svm_train(labels, features, '-c 4')
#problem = svm_problem(labels,features)
#svm = cv2.ml()
#svm.train(features,labels, params=svm_params)
#svm.save('svm_data.dat')


labelsList = labels.tolist()
#labelsList = float(labelsList)
featuresList = features.tolist()
#featuresList = float(featuresList)

print(type(featuresList[1][0]))

#prob = svm_problem(labelsList,featuresList)

#m = svms.svm_train(labelsList,featuresList,"-s 1 -t 2 -n 0.25 -g 0.00001")
#svms.svm_save_model('myModel.xml',m)
m = svms.svm_load_model('myModel.xml')
p_label, p_acc, p_val = svms.svm_predict(labelsList, featuresList, m)

print "Accuracy is ", p_acc



'''
#Test Code 

hog = cv2.HOGDescriptor()
img = np.zeros((512,512,3), np.uint8)
#cv2.imwrite("black.jpg",img)
im = cv2.imread("poster.jpg")
h = hog.compute(im)
h = np.transpose(h)
h = np.concatenate((h,h), axis = 0)
print h.shape
'''