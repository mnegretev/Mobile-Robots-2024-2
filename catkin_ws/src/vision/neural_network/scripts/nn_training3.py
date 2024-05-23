#!/usr/bin/env python3
#
# MOBILE ROBOTS - FI-UNAM, 2024-2
# TRAINING A NEURAL NETWORK
#
# Instructions:
# Complete the code to train a neural network for
# handwritten digits recognition.
#
import cv2
import sys
import random
import numpy
import rospy
import rospkg
import itertools
import csv
import os
import time

NAME = "BAFO INC. SEMICONDUCTORS"

class NeuralNetwork(object):
    def __init__(self, layers, weights=None, biases=None):
        #
        # The list 'layers' indicates the number of neurons in each layer.
        # Remember that the first layer indicates the dimension of the inputs and thus,
        # there is no bias vector fot the first layer.
        # For this practice, 'layers' should be something like [784, n2, n3, ..., nl, 10]
        # All weights and biases are initialized with random values. In each layer we have a matrix
        # of weights where row j contains all the weights of the j-th neuron in that layer. For this example,
        # the first matrix should be of order n2 x 784 and last matrix should be 10 x nl.
        #
        self.num_layers  = len(layers)
        self.layer_sizes = layers
        self.biases =[numpy.random.randn(y,1) for y in layers[1:]] if biases == None else biases
        self.weights=[numpy.random.randn(y,x) for x,y in zip(layers[:-1],layers[1:])] if weights==None else weights
        
    def feedforward(self, x):
        #
        # This function gets the output of the network when input is 'x'.
        #
        for i in range(len(self.biases)):
            z = numpy.dot(self.weights[i], x) + self.biases[i]
            x = 1.0 / (1.0 + numpy.exp(-z))  #output of the current layer is the input of the next one
        return x

    def feedforward_verbose(self, x):
        #y = []
        #
        # TODO: CODED BY HERO 7W7
        # Write a function similar to 'feedforward' but instead of returning only the output layer,
        # return a list containing the output of each layer, from input to output.
        # Include input x as the first output.
        #
        y = [x]
        for i in range(len(self.biases)):
            z = numpy.dot(self.weights[i],x) + self.biases[i]
            x = 1.0/ (1.0 + numpy.exp(-z))
            y.append(x)
        return y


    def backpropagate(self, x, yt):
        y = self.feedforward_verbose(x)
        nabla_b = [numpy.zeros(b.shape) for b in self.biases]
        nabla_w = [numpy.zeros(w.shape) for w in self.weights]
        # TODO:
        # Return a tuple [nabla_w, nabla_b] containing the gradient of cost function C with respect to
        # each weight and bias of all the network. The gradient is calculated assuming only one training
        # example is given: the input 'x' and the corresponding label 'yt'.
        # nabla_w and nabla_b should have the same dimensions as the corresponding
        # self.weights and self.biases
        # You can calculate the gradient following these steps:
        #
        # Calculate delta for the output layer L: delta=(yL-yt)*yL*(1-yL)
        # nabla_b of output layer = delta      
        # nabla_w of output layer = delta*yLpT where yLpT is the transpose of the ouput vector of layer L-1
        # FOR all layers 'l' from L-1 to input layer: 
        #     delta = (WT * delta)*yl*(1 - yl)
        #     where 'WT' is the transpose of the matrix of weights of layer l+1 and 'yl' is the output of layer l
        #     nabla_b[-l] = delta
        #     nabla_w[-l] = delta*ylpT  where ylpT is the transpose of outputs vector of layer l-1
        delta = (y[-1] - yt) * y[-1] * (1 - y[-1])
        nabla_b[-1] = delta
        nabla_w[-1] = numpy.dot(delta, y[-2].T)
	
        for l in range(2, len(self.biases)+ 1):
            z = numpy.dot(self.weights[-l + 1 ].T, delta)   
            delta = z * y[-l] * (1-y[-l])
            nabla_b[-l] = delta
            nabla_w[-l] = numpy.dot(delta, y[-l-1].T)
        
        return nabla_w, nabla_b

    def update_with_batch(self, batch, eta):
        #
        # This function exectutes gradient descend for the subset of examples
        # given by 'batch' with learning rate 'eta'
        # 'batch' is a list of training examples [(x,y), ..., (x,y)]
        #
        nabla_b = [numpy.zeros(b.shape) for b in self.biases]
        nabla_w = [numpy.zeros(w.shape) for w in self.weights]
        M = len(batch)
        for x,y in batch:
            if rospy.is_shutdown():
                break
            delta_nabla_w, delta_nabla_b = self.backpropagate(x,y)
            nabla_w = [nw+dnw for nw,dnw in zip(nabla_w, delta_nabla_w)]
            nabla_b = [nb+dnb for nb,dnb in zip(nabla_b, delta_nabla_b)]
        self.weights = [w-eta*nw/M for w,nw in zip(self.weights, nabla_w)]
        self.biases  = [b-eta*nb/M for b,nb in zip(self.biases , nabla_b)]
        return nabla_w, nabla_b

    def get_gradient_mag(self, nabla_w, nabla_b):
        mag_w = sum([numpy.sum(n) for n in [nw*nw for nw in nabla_w]])
        mag_b = sum([numpy.sum(b) for b in [nb*nb for nb in nabla_b]])
        return mag_w + mag_b

    def train_by_SGD(self, training_data, epochs, batch_size, eta):
        for j in range(epochs):
            random.shuffle(training_data)
            batches = [training_data[k:k+batch_size] for k in range(0,len(training_data), batch_size)]
            for batch in batches:
                if rospy.is_shutdown():
                    return
                nabla_w, nabla_b = self.update_with_batch(batch, eta)
                sys.stdout.write("\rGradient magnitude: %f            " % (self.get_gradient_mag(nabla_w, nabla_b)))
                sys.stdout.flush()
            print("Epoch: " + str(j))
    #
    ### END OF CLASS
    #


def load_dataset(folder):
    print("Loading data set from " + folder)
    if not folder.endswith("/"):
        folder += "/"
    training_dataset, training_labels, testing_dataset, testing_labels = [],[],[],[]
    for i in range(10):
        f_data = [c/255.0 for c in open(folder + "data" + str(i), "rb").read(784000)]
        images = [numpy.asarray(f_data[784*j:784*(j+1)]).reshape([784,1]) for j in range(1000)]
        label  = numpy.asarray([1 if i == j else 0 for j in range(10)]).reshape([10,1])
        training_dataset += images[0:len(images)//2]
        training_labels  += [label for j in range(len(images)//2)]
        testing_dataset  += images[len(images)//2:len(images)]
        testing_labels   += [label for j in range(len(images)//2)]
    return list(zip(training_dataset, training_labels)), list(zip(testing_dataset, testing_labels))


def save_results_to_csv(results, filename='training_results.csv'):
    file_exists = os.path.isfile(filename)
    with open(filename, mode='a', newline='') as file:
        writer = csv.writer(file)
        if not file_exists:
            writer.writerow(['epochs', 'batch_size', 'learning_rate', 'y_count', 'n_count', 'porc_exitos', 'time_exec'])
        for result in results:
            writer.writerow(result)


def main():
    print("TRAINING A NEURAL NETWORK - " + NAME)
    rospy.init_node("nn_training3")
    rospack = rospkg.RosPack()
    dataset_folder = rospack.get_path("neural_network") + "/handwritten_digits/"
    
    # epochs_list = [3, 10, 50, 100]
    # epochs_list = [3, 10] # GADIEL
    # epochs_list = [50, 100] # MAURICIO
    epochs_list = [3] # MAURICIO
    batch_size_list = [5, 10, 30, 100]
    learning_rate_list = [0.5, 1.0, 3.0, 10.0]
    
    iteraciones = 100

    for epochs, batch_size, learning_rate in itertools.product(epochs_list, batch_size_list, learning_rate_list):
        print("Parámetros actuales:")
        print(f"epochs: {epochs}, batch_size: {batch_size}, learning_rate: {learning_rate}")
        # epochs        = *
        # batch_size    = *
        # learning_rate = *
        
        training_dataset, testing_dataset = load_dataset(dataset_folder)
        
        try:
            saved_data = numpy.load(dataset_folder+"network.npz",allow_pickle=True)
            layers = [saved_data['w'][0].shape[1]] + [b.shape[0] for b in saved_data['b']]
            nn = NeuralNetwork(layers, weights=saved_data['w'], biases=saved_data['b'])
            print("Loading data from previously trained model with layers " + str(layers))
        except:
            nn = NeuralNetwork([784,30,10])
            pass
        time_ini = time.time()
        nn.train_by_SGD(training_dataset, epochs, batch_size, learning_rate)
        time_fin = time.time()
        time_exec = time_fin - time_ini
        print(f"time_exec: {time_exec}")
        #numpy.savez(dataset_folder + "network",w=nn.weights, b=nn.biases)
        
        # print("\nPress key to test network or ESC to exit...")
        numpy.set_printoptions(formatter={'float_kind':"{:.3f}".format})
        cmd = cv2.waitKey(0)

        # while cmd != 27 and not rospy.is_shutdown():
        results = []
        y_count = 0
        n_count = 0
        for i in range(iteraciones):    
            img,label = testing_dataset[numpy.random.randint(0, 4999)]
            y = nn.feedforward(img).transpose()
            print("\nPerceptron output: " + str(y))
            print("Expected output  : "   + str(label.transpose()))
            print("Recognized digit : "   + str(numpy.argmax(y)))
            cv2.imshow("Digit", numpy.reshape(numpy.asarray(img, dtype="float32"), (28,28,1)))
            cmd = cv2.waitKey(100) # Genera la imagen
            
            user_input = input("Enter 'y' or 'n': ")
            if user_input == 'y':
                y_count += 1
            elif user_input == 'n':
                n_count += 1
            print("END OF IT:", i+1)
        porc_exitos = int((y_count / iteraciones) * 100)
        results.append((epochs, batch_size, learning_rate, y_count, n_count, porc_exitos, time_exec))
        save_results_to_csv(results)
        print("Count of 'y':", y_count)
        print("Count of 'n':", n_count)
        print("END OF ITERACIONES")
    

if __name__ == '__main__':
    main()

