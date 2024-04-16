import numpy as np

# Define the AND gate inputs and outputs
X = np.array([[0, 0],
              [0, 1],
              [1, 0],
              [1, 1]])

y = np.array([0, 0, 0, 1])

# Initialize weights and bias
w = np.array([0.0, 0.0])
b = 0.0

# Learning rate
lr = 0.1


# Define the sigmoid activation function
def sigmoid(x):
    return 1 / (1 + np.exp(-x))


# Calculate the gradients for one iteration
def calculate_gradients(X, y, w, b):
    m = len(X)
    y_pred = sigmoid(np.dot(X, w) + b)
    error = y_pred - y
    dw = np.dot(X.T, error) / m
    db = np.sum(error) / m
    return dw, db


# Update weights and bias using gradients
def update_parameters(w, b, dw, db, lr):
    w -= lr * dw
    b -= lr * db
    return w, b


# Perform two iterations of gradient descent
for i in range(2):
    print(f"Iteration {i + 1}:")
    print("x1\tx2\ty1\tu\tGradients\tPrediction\tPartial of w1\tPartial of w2\tPartial of bias")

    for j in range(len(X)):
        x = X[j]
        y_true = y[j]

        # Forward pass
        u = np.dot(x, w) + b
        y_pred = sigmoid(u)

        # Calculate gradients
        dw, db = calculate_gradients(np.expand_dims(x, axis=0), y_true, w, b)

        # Update parameters
        w, b = update_parameters(w, b, dw, db, lr)

        print(f"{x[0]}\t{x[1]}\t{y_true}\t{u:.2f}\t{dw}\t\t{y_pred:.2f}\t\t{dw[0]:.2f}\t\t{dw[1]:.2f}\t\t{db:.2f}")

    print()
