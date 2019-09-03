import glob
import dlib
import cv2
import pickle
import random
import face
import numpy as np


def adjust_gamma(input_image, gamma=1.0):
    table = np.array([((iteration / 255.0) ** (1.0 / gamma)) * 255
                      for iteration in np.arange(0, 256)]).astype("uint8")
    return cv2.LUT(input_image, table)


def read_image(path, gamma=0.75):
    output = cv2.imread(path)
    return adjust_gamma(output, gamma=gamma)


def face_vector(input_image):
    faces = face.detector(input_image, 1)
    if not faces:
        return None

    f = faces[0]
    shape = face.predictor(input_image, f)
    face_descriptor = face.face_model.compute_face_descriptor(input_image, shape)
    return face_descriptor


max_size = 340
male_label = +1
female_label = -1

print "Retrieving males images ..."
males = glob.glob("./imdb-datasets/images/males/*.jpg")
print "Retrieved {} faces !".format(len(males))

print "Retrieving females images ..."
females = glob.glob("./imdb-datasets/images/females/*.jpg")
print "Retrieved {} faces !".format(len(females))

females = females[:max_size]
males = males[:max_size]

vectors = dlib.vectors()
labels = dlib.array()

print "Reading males images ..."
for i, male in enumerate(males):
    print "Reading {} of {}\r".format(i, len(males))
    face_vectors = face_vector(read_image(male))
    if face_vectors is None:
        continue
    vectors.append(dlib.vector(face_vectors))
    labels.append(male_label)

print "Reading females images ..."
for i, female in enumerate(females):
    print "Reading {} of {}\r".format(i, len(females))
    face_vectors = face_vector(read_image(female))
    if face_vectors is None:
        continue
    vectors.append(dlib.vector(face_vectors))
    labels.append(female_label)

svm = dlib.svm_c_trainer_radial_basis()
svm.set_c(10)
classifier = svm.train(vectors, labels)

print "Prediction for male sample:  {}".format(classifier(vectors[random.randrange(0, max_size)]))
print "Prediction for female sample: {}".format(classifier(vectors[max_size + random.randrange(0, max_size)]))

with open('gender_model.pickle', 'wb') as handle:
    pickle.dump(classifier, handle)
