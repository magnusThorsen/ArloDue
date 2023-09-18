import cv2

# Load the image
image = cv2.imread('your_image_with_qr_code.jpg')

# Create a QRCodeDetector object
qr_code_detector = cv2.QRCodeDetector()

# Detect QR code(s) in the image
retval, decoded_info, points, straight_qrcode = qr_code_detector.detectAndDecodeMulti(image)

# Check if a QR code was found
if retval:
    # Measure the size of the first detected QR code
    if len(points) > 0:
        # Calculate the width and height of the QR code
        width = abs(points[0][0][0] - points[0][2][0])
        height = abs(points[0][0][1] - points[0][2][1])
        
        print(f"QR Code Size (Width x Height): {width:.2f} x {height:.2f} pixels")
    else:
        print("No valid QR code found in the image.")
else:
    print("No QR code found in the image.")

# Display the image with the detected QR code(s)
cv2.imshow('Image with QR Code', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
