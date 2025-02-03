# test_tflite.py
import tflite_runtime.interpreter as tflite # type: ignore

# Load the TFLite model and allocate tensors.
interpreter = tflite.Interpreter(model_path="your_model.tflite")
interpreter.allocate_tensors()

# Get input and output tensors.
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

print("Input details:", input_details)
print("Output details:", output_details)