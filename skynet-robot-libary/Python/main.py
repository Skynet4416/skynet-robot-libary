import numpy as np
import tensorflow as tf
import autokeras as ak

def get_input(file):
    return_x = []
    return_y = []
    with open(file,"r") as read_file:
        lines = read_file.read().split("\n")[1::]
        for line in lines:
            if(line != ''):
                elements = line.split(",")
                return_x.append(float(elements[0]))
                return_y.append([float(elements[1]),float(elements[2]),float(elements[3])])
    
    return np.asarray(return_x).reshape(len(return_x),1),np.asarray(return_y)


training = True

reg = ak.StructuredDataRegressor(overwrite=False, max_trials=500)

input_train,output_train = get_input("optimization_train.csv")
input_test,output_test = get_input("optimization_test.csv")

try:
    reg = ak.load_model("model_autokeras", custom_objects=ak.CUSTOM_OBJECTS)
except:
    pass

if(training):
    # Train the model
    reg.fit(input_train,output_train, epochs=1000, validation_split=0.2) 

    # Get the model's metrics
    print("PREDICTING")
    predicted = reg.predict(input_train) 

    # Evaluate
    print(reg.evaluate(input_train,output_train))

    # Save the model
    model = reg.export_model() 
    model.summary()


input = np.asarray([2.5])
print(reg.predict(input)) 