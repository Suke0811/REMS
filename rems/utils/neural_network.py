import numpy as np
from sklearn.datasets import make_moons

class NeuralNetwork():
    def __init__(self, NN_ARCHITECTURE, load_params):
        self.nn_architecture = NN_ARCHITECTURE
        self.seed = 2
        if len(load_params) > 0:
            self.params_values = self.init_layers()
            self.params_values = self.Auto_to_Neural_Format(load_params)
        else:
            self.params_values = self.init_layers()

    def init_layers(self):
        # random seed initiation
        np.random.seed(self.seed)
        # parameters storage initiation
        params_values = {}

        # iteration over network layers
        for idx, layer in enumerate(self.nn_architecture):
            # we number network layers from 1
            layer_idx = idx + 1

            # extracting the number of units in layers
            layer_input_size = layer["input_dim"]
            layer_output_size = layer["output_dim"]

            # initiating the values of the W matrix
            # and vector b for subsequent layers
            params_values['W' + str(layer_idx)] = np.random.randn(
                layer_output_size, layer_input_size) * 0.1 # 0.01
            params_values['b' + str(layer_idx)] = np.random.randn(
                layer_output_size, 1) * 0.1 # 0.01

        return params_values

    def sigmoid(self, Z):
        return 1 / (1 + np.exp(-Z))

    def relu(self, Z):
        return np.maximum(0, Z)

    def leakyRelu(self, Z):
        return np.maximum(0.001*Z,Z) # 0.001

    def linear(self, Z):
        return Z

    def single_layer_forward_propagation(self, A_prev, W_curr, b_curr, activation="relu"):
        # calculation of the input value for the activation function
        Z_curr = np.dot(W_curr, A_prev) + b_curr

        # selection of activation function
        if activation == "relu":
            activation_func = self.relu
        elif activation == "sigmoid":
            activation_func = self.sigmoid
        elif activation == "leakyRelu":
            activation_func = self.leakyRelu
        elif activation == "linear":
            activation_func = self.linear
        else:
            raise Exception('Non-supported activation function')

        # return of calculated activation A and the intermediate Z matrix
        return activation_func(Z_curr), Z_curr

    def full_forward_propagation(self, X):
        # creating a temporary memory to store the information needed for a backward step
        memory = {}
        # X vector is the activation for layer
        A_curr = X

        # iteration over network layers
        for idx, layer in enumerate(self.nn_architecture):
            # we number network layers from 1
            layer_idx = idx + 1
            # transfer the activation from the previous iteration
            A_prev = A_curr

            # extraction of the activation function for the current layer
            activ_function_curr = layer["activation"]
            # extraction of W for the current layer
            W_curr = self.params_values["W" + str(layer_idx)]
            # extraction of b for the current layer
            b_curr = self.params_values["b" + str(layer_idx)]
            # calculation of activation for the current layer
            A_curr, Z_curr = self.single_layer_forward_propagation(A_prev, W_curr, b_curr, activ_function_curr)

            # saving calculated values in the memory
            memory["A" + str(idx)] = A_prev
            memory["Z" + str(layer_idx)] = Z_curr

        # return of prediction vector and a dictionary containing intermediate values
        return A_curr, memory

    def Neural_to_Auto_Format(self, params_values):
        # input is the params_values dictionary, and the output is a single array that contains all the weights for auto tuning
        param_auto_tune = []

        for i in range(len(params_values)):
            param = list(params_values.values())[i]
            param_reshape = param.reshape(param.shape[0]*param.shape[1],).tolist()
            param_auto_tune = param_auto_tune + param_reshape

        param_auto_tune = np.asarray(param_auto_tune).reshape(len(param_auto_tune),1)

        return param_auto_tune

    def Auto_to_Neural_Format(self, param_auto_tune):
        # inputs is a single array that contains all the weights for auto tuning, and the output is the params_values dictionary used for the neural network
        params_values = self.params_values
        index_counter = 0
        for i in range(len(params_values)):
            param = list(params_values.values())[i]
            cur_indexes = param.shape[0]*param.shape[1]
            values = np.array(param_auto_tune[index_counter:index_counter+cur_indexes,]).reshape(param.shape[0],param.shape[1])
            params_values[list(params_values.keys())[i]] = values
            index_counter += cur_indexes

        return params_values


if __name__ == '__main__':
    NN_ARCHITECTURE = [
        {"input_dim": 2, "output_dim": 25, "activation": "relu"},
        {"input_dim": 25, "output_dim": 50, "activation": "relu"},
        {"input_dim": 50, "output_dim": 50, "activation": "relu"},
        {"input_dim": 50, "output_dim": 25, "activation": "relu"},
        {"input_dim": 25, "output_dim": 3, "activation": "sigmoid"},
    ]
    # number of samples in the data set
    N_SAMPLES = 1
    # ratio between training and test sets
    TEST_SIZE = 0.1
    NN = NeuralNetwork(NN_ARCHITECTURE, [])
    X, y = make_moons(n_samples=N_SAMPLES, noise=0.2, random_state=100)
    #X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=TEST_SIZE, random_state=42)
    Y_hat, _ = NN.full_forward_propagation(np.transpose(X))
    print(Y_hat)
    param_auto_tune = NN.Neural_to_Auto_Format(NN.params_values)
    params_value = NN.Auto_to_Neural_Format(param_auto_tune)
