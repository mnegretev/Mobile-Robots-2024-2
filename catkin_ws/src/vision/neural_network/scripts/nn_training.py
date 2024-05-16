#!/usr/bin/env python3
#
# MOBILE ROBOTS - FI-UNAM, 2024-2
# TRAINING A NEURAL NETWORK
#
# Instructions:
# Complete the code to train a neural network for
# handwritten digits recognition.
#

import numpy
import cv2
import sys
import random
import time
import rospy
import rospkg

NAME = "LUNA COLMENARES MIGUEL ANGEL"

class NeuralNetwork(object):
    def __init__(self, layers, weights=None, biases=None):
        self.num_layers = len(layers)
        self.layer_sizes = layers
        self.biases = [numpy.random.randn(y, 1) for y in layers[1:]] if biases is None else biases
        self.weights = [numpy.random.randn(y, x) for x, y in zip(layers[:-1], layers[1:])] if weights is None else weights

    def feedforward(self, x):
        for i in range(len(self.biases)):
            z = numpy.dot(self.weights[i], x) + self.biases[i]
            x = 1.0 / (1.0 + numpy.exp(-z))
        return x

    def feedforward_verbose(self, x):
        y = [x]
        for i in range(len(self.biases)):
            u = numpy.dot(self.weights[i], x) + self.biases[i]
            x = 1.0 / (1.0 + numpy.exp(-u))
            y.append(x)
        return y

    def backpropagate(self, x, t):
        y = self.feedforward_verbose(x)
        nabla_b = [numpy.zeros(b.shape) for b in self.biases]
        nabla_w = [numpy.zeros(w.shape) for w in self.weights]
        delta = (y[-1] - t) * y[-1] * (1 - y[-1])
        nabla_w[-1] = delta * y[-2].T
        nabla_b[-1] = delta
        for i in range(2, self.num_layers):
            delta = numpy.dot(self.weights[-i+1].T, delta) * y[-i] * (1 - y[-i])
            nabla_w[-i] = delta * y[-i-1].T
            nabla_b[-i] = delta
        return nabla_w, nabla_b

    def update_with_batch(self, batch, eta):
        nabla_b = [numpy.zeros(b.shape) for b in self.biases]
        nabla_w = [numpy.zeros(w.shape) for w in self.weights]
        M = len(batch)
        for x, y in batch:
            if rospy.is_shutdown():
                break
            delta_nabla_w, delta_nabla_b = self.backpropagate(x, y)
            nabla_w = [nw+dnw for nw, dnw in zip(nabla_w, delta_nabla_w)]
            nabla_b = [nb+dnb for nb, dnb in zip(nabla_b, delta_nabla_b)]
        self.weights = [w-eta*nw/M for w, nw in zip(self.weights, nabla_w)]
        self.biases = [b-eta*nb/M for b, nb in zip(self.biases, nabla_b)]
        return nabla_w, nabla_b

    def get_gradient_mag(self, nabla_w, nabla_b):
        mag_w = sum(numpy.sum(n**2) for n in nabla_w)
        mag_b = sum(numpy.sum(b**2) for b in nabla_b)
        return mag_w + mag_b

    def train_by_SGD(self, training_data, epochs, batch_size, eta):
        for j in range(epochs):
            random.shuffle(training_data)
            batches = [training_data[k:k+batch_size] for k in range(0, len(training_data), batch_size)]
            for batch in batches:
                if rospy.is_shutdown():
                    return
                nabla_w, nabla_b = self.update_with_batch(batch, eta)
                sys.stdout.write("\rGradient magnitude: %f            " % (self.get_gradient_mag(nabla_w, nabla_b)))
                sys.stdout.flush()
            print("Epoch: " + str(j))

def load_dataset(folder):
    print("Loading data set from " + folder)
    if not folder.endswith("/"):
        folder += "/"
    training_dataset, training_labels, testing_dataset, testing_labels = [], [], [], []
    for i in range(10):
        f_data = [c/255.0 for c in open(folder + "data" + str(i), "rb").read(784000)]
        images = [numpy.asarray(f_data[784*j:784*(j+1)]).reshape([784, 1]) for j in range(1000)]
        label = numpy.asarray([1 if i == j else 0 for j in range(10)]).reshape([10, 1])
        training_dataset += images[0:len(images)//2]
        training_labels += [label for j in range(len(images)//2)]
        testing_dataset += images[len(images)//2:len(images)]
        testing_labels += [label for j in range(len(images)//2)]
    return list(zip(training_dataset, training_labels)), list(zip(testing_dataset, testing_labels))



def main():
    print("TRAINING A NEURAL NETWORK - " + NAME)
    rospy.init_node("nn_training")
    rospack = rospkg.RosPack()
    dataset_folder = rospack.get_path("neural_network") + "/handwritten_digits/"
    
    training_dataset, testing_dataset = load_dataset(dataset_folder)
    
    learning_rates = [0.5, 1.0, 3.0,10.0]
    epoch_sets = [3, 10, 50,100]
    batch_sizes = [5, 10, 30,100]
    i = 100

    for eta in learning_rates:
        for epochs in epoch_sets:
            for batch_size in batch_sizes:
                times = []
                accuracies = []
                for _ in range(i):
                    start_time = rospy.Time.now()
                    nn = NeuralNetwork([784, 30, 10])
                    nn.train_by_SGD(training_dataset, epochs, batch_size, eta)
                    end_time = rospy.Time.now()
                    duration = (end_time - start_time).to_sec()
                    times.append(duration)

                    hint = 0
                    total = len(testing_dataset)
                    for img, label in testing_dataset:
                        output = nn.feedforward(img)
                        if numpy.argmax(output) == numpy.argmax(label):
                            hint += 1

                    accuracy = hint / total
                    accuracies.append(accuracy)

                average_time = sum(times) / i
                average_accuracy = sum(accuracies) / i
           
                print(f"Learning Rate: {eta}, Epochs: {epochs}, Batch Size: {batch_size}, Avg Time: {average_time:.2f}s, Avg Accuracy: {average_accuracy:.2%}")

if __name__=='__main__':
    main()
