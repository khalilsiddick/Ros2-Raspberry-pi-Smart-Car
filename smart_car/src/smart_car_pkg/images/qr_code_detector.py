import cv2

# Read the image from file
img = cv2.imread('right.png')

# Create a QRCodeDetector object
decoder = cv2.QRCodeDetector()

# Detect and decode the QR code
data, points, binary_qr_code = decoder.detectAndDecode(img)

    
print("Result = ", data)
        

# Display the image with the detected QR code
cv2.imshow('Detected QR code is', img)

# Wait for a key press
cv2.waitKey()

# Destroy all OpenCV windows
cv2.destroyAllWindows()