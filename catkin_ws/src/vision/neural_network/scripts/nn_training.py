#!/usr/bin/env python3

import cv2
import sys
import random
import numpy as np
import rospy
import rospkg
import time
import itertools

NAME = "Roberto Camacho"

class NeuralNetwork(object):
    def __init__(self, layers, weights=None, biases=None):
        if layers[0] != 784:
            raise ValueError("First layer size should match input image size (784)")

        self.num_layers = len(layers)
        self.layer_sizes = layers
        self.biases = [np.random.randn(y, 1) for y in layers[1:]] if biases is None else biases
        self.weights = [
            np.random.randn(y, x) for x, y in zip(layers[:-1], layers[1:])
        ] if weights is None else weights
        
    def feedforward(self, x):
        for i in range(len(self.biases)):
            z = np.dot(self.weights[i], x) + self.biases[i]
            x = 1.0 / (1.0 + np.exp(-z))
        return x

    def feedforward_verbose(self, x):
        outputs = [x]
        for i in range(len(self.biases)):
            z = np.dot(self.weights[i], x) + self.biases[i]
            x = 1.0 / (1.0 + np.exp(-z))
            outputs.append(x)
        return outputs

    def backpropagate(self, x, yt):
        delta = (yt - self.feedforward(x)) * self.feedforward(x) * (1 - self.feedforward(x))
        nabla_b = [delta]
        nabla_w = [np.dot(delta, x.T)]

        for l in range(len(self.biases) - 2, -1, -1):
            y = self.feedforward_verbose(x)[l + 1]
            delta = np.dot(self.weights[l + 1].T, delta) * y * (1 - y)
            nabla_b.append(delta)
                
            nabla_w.append(np.dot(delta, self.feedforward_verbose(x)[l].T))

        return nabla_w[::-1], nabla_b[::-1]

    def update_with_batch(self, batch, eta):
        # Update weights and biases using batch gradient descent
        nabla_w = [np.zeros(w.shape) for w in self.weights]
        nabla_b = [np.zeros(b.shape) for b in self.biases]
        M = len(batch)
        for x, y in batch:
            if rospy.is_shutdown():
                break
            delta_nabla_w, delta_nabla_b = self.backpropagate(x, y)

            for i in range(len(nabla_w)):
                # Ajustar la forma de delta_nabla_w si es necesario
                if delta_nabla_w[i].shape != nabla_w[i].shape:
                    delta_nabla_w[i] = delta_nabla_w[i][:nabla_w[i].shape[0], :nabla_w[i].shape[1]]
                nabla_w[i] += delta_nabla_w[i]
            nabla_b = [nb + dnb for nb, dnb in zip(nabla_b, delta_nabla_b)]

        self.weights = [w - (eta / M) * nw for w, nw in zip(self.weights, nabla_w)]
        self.biases = [b - (eta / M) * nb for b, nb in zip(self.biases, nabla_b)]
        return nabla_w, nabla_b

    def get_gradient_mag(self, nabla_w, nabla_b):
        mag_w = sum([np.sum(n) for n in [nw*nw for nw in nabla_w]])
        mag_b = sum([np.sum(b) for b in [nb*nb for nb in nabla_b]])
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

def load_dataset(folder):
    print("Loading data set from " + folder)
    if not folder.endswith("/"):
        folder += "/"
    training_dataset, training_labels, testing_dataset, testing_labels = [],[],[],[]
    for i in range(10):
        f_data = [c/255.0 for c in open(folder + "data" + str(i), "rb").read(784000)]
        images = [np.asarray(f_data[784*j:784*(j+1)]).reshape([784,1]) for j in range(1000)]
        label  = np.asarray([1 if i == j else 0 for j in range(10)]).reshape([10,1])
        training_dataset += images[0:len(images)//2]
        training_labels  += [label for j in range(len(images)//2)]
        testing_dataset  += images[len(images)//2:len(images)//2+50]  # Tomar solo las primeras 50 imágenes para el conjunto de prueba
        testing_labels   += [label for j in range(len(images)//2)]
    return list(zip(training_dataset, training_labels)), list(zip(testing_dataset, testing_labels))

def test_network(nn, testing_dataset, num_tests=500):
    total_samples = len(testing_dataset)
    total_accuracy = 0
    for _ in range(num_tests):
        correct_predictions = 0
        for img, label in testing_dataset:
            y = nn.feedforward(img)
            expected = np.argmax(label)
            recognized = np.argmax(y)
            if expected == recognized:
                correct_predictions += 1
        accuracy = (correct_predictions / total_samples) * 100
        total_accuracy += accuracy
    # Imprimimos el número de pruebas y predicciones correctas
    print("Number of tests:", num_tests)
    print("Correct predictions:", correct_predictions)
    average_accuracy = total_accuracy / num_tests
    return average_accuracy

def main():
    print("TRAINING A NEURAL NETWORK - Roberto C")
    rospy.init_node("nn_training")
    rospack = rospkg.RosPack()
    dataset_folder = rospack.get_path("neural_network") + "/handwritten_digits/"
    
    # Definir los valores a probar para cada parámetro
    learning_rates = [0.5, 1.0, 3.0, 10.0]
    epochs_values = [3, 10, 50, 100]
    batch_sizes = [5, 10, 30, 100]

    # Combinar cada elemento de las listas
    combinations = itertools.product(learning_rates, epochs_values, batch_sizes)
    
    combination_count = 0  # Contador para el número de combinaciones

    print("Starting experiments...\n")

    # Realizar pruebas para cada combinación de parámetros
    for learning_rate, epochs, batch_size in combinations:
        combination_count += 1  # Incrementar el contador de combinaciones
        print(f"Combination {combination_count}:")
        print(f"    Training with learning rate: {learning_rate}, epochs: {epochs}, batch size: {batch_size}")
        
        try:
            # Cargar conjunto de datos
            training_dataset, testing_dataset = load_dataset(dataset_folder)
            
            # Crear instancia de la red neuronal
            nn = NeuralNetwork([784, 30, 10])
            
            # Entrenar la red neuronal
            start_time = time.time()
            nn.train_by_SGD(training_dataset, epochs, batch_size, learning_rate)
            end_time = time.time()
            
            # Realizar pruebas con la red neuronal entrenada
            hits = test_network(nn, testing_dataset)
            
            print(f"    Average training time: {end_time - start_time:.2f} seconds")
            print(f"    Average accuracy: {hits:.2f}%\n")  # Cambio en la impresión del porcentaje de precisión
            
        except Exception as e:
            print(f"    An error occurred: {str(e)}\n")

    print(f"Total number of combinations: {combination_count}")

if __name__ == "__main__":
    main()